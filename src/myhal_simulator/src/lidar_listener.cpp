#include "lidar_listener.hh"
#include <string>


GZ_REGISTER_WORLD_PLUGIN(LidarListener);

//void Callback(const PointCloud::ConstPtr& msg);

void LidarListener::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){
    this->world = _world;
    this->sdf = _sdf;
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&LidarListener::OnUpdate, this, std::placeholders::_1));

    this->ReadSDF();

    auto building = this->world->ModelByName(this->building_name);

    this->building_box = building->BoundingBox();
    this->building_box.Min().X()-=1;
    this->building_box.Min().Y()-=1;
    this->building_box.Max().X()+=1;
    this->building_box.Max().Y()+=1;
    this->static_quadtree = boost::make_shared<QuadTree>(this->building_box);
    this->vehicle_quadtree = boost::make_shared<QuadTree>(this->building_box);

    

    for (unsigned int i = 0; i < world->ModelCount(); ++i) {
        auto model = world->ModelByIndex(i);
        auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

        if (act){
            this->actors.push_back(act);
            auto actor_pos = act->WorldPose().Pos();
            auto min = ignition::math::Vector3d(actor_pos.X() - 0.25, actor_pos.Y() - 0.25, 0);
            auto max = ignition::math::Vector3d(actor_pos.X() + 0.25, actor_pos.Y() + 0.25, 0);
            auto box = ignition::math::Box(min,max);
            auto new_node = QTData(box, act, entity_type);
            this->vehicle_quadtree->Insert(new_node);
            continue;
        } 
      
        if (model->GetName() == this->building_name){
            auto links = model->GetLinks();
            for (gazebo::physics::LinkPtr link: links){
                std::vector<gazebo::physics::CollisionPtr> collision_boxes = link->GetCollisions();
                for (gazebo::physics::CollisionPtr collision_box: collision_boxes){
                    this->building_collisions.push_back(collision_box); //TODO: check if this is correct (maybe do dynamic pointer cast )
                    auto new_node = QTData(collision_box->BoundingBox(), collision_box, entity_type);
                    this->static_quadtree->Insert(new_node);
                }
                    
            }
        } else if (model->GetName() != "ground_plane"){
            auto links = model->GetLinks();
            for (gazebo::physics::LinkPtr link: links){
                
                this->model_collisions.push_back(link); //TODO: check if this is correct (maybe do dynamic pointer cast )
                auto box = link->BoundingBox();
                box.Min().Z() = 0;
                box.Max().Z() = 0;
                auto new_node = QTData(box, link, entity_type);
                this->static_quadtree->Insert(new_node);
            }
            
        } 
        
        
    }

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "LidarListener");
    
    this->sub = this->nh.subscribe<PointCloud>("velodyne_points", 1, &LidarListener::Callback, this);
    this->ground_pub = nh.advertise<PointCloud>("ground_points", 1000);
    this->wall_pub = nh.advertise<PointCloud>("wall_points", 1000);
    this->actor_pub = nh.advertise<PointCloud>("actor_points", 1000);
    this->table_pub = nh.advertise<PointCloud>("table_points", 1000);
    this->chair_pub = nh.advertise<PointCloud>("chair_points", 1000);
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    ros::Rate r = 10;
    spinner.start();
}

void LidarListener::OnUpdate(const gazebo::common::UpdateInfo &_info){
    double dt = (_info.simTime - this->last_update).Double();

    if (dt < 1/this->update_freq){
        return;
    }

    this->last_update = _info.simTime;

    if ((this->robot_name != "") && this->robot == nullptr){
        for (unsigned int i = 0; i < world->ModelCount(); ++i) {
            auto model = world->ModelByIndex(i);
            if (model->GetName() == this->robot_name){
                this->robot = model;
                
                std::cout << "ADDED ROBOT: " << this->robot->GetName() << std::endl;
                this->robot_links = this->robot->GetLinks();
              
            }
        }
    } 

    if (this->robot != nullptr){
    
        this->sensor_pose = this->robot_links[0]->WorldPose();
        this->sensor_pose.Pos().Z() += 0.5767;
    }


   
}

void LidarListener::Callback(const PointCloud::ConstPtr& msg){

    if (this->robot == nullptr){
        return;
    }

    PointCloud::Ptr ground_msg (new PointCloud);
    ground_msg->header.frame_id = "velodyne";
    ground_msg->height = msg->height;
    ground_msg->width = 0;

    PointCloud::Ptr wall_msg (new PointCloud);
    wall_msg->header.frame_id = "velodyne";
    wall_msg->height = msg->height;
    wall_msg->width = 0;
    
    PointCloud::Ptr actor_msg (new PointCloud);
    actor_msg->header.frame_id = "velodyne";
    actor_msg->height = msg->height;
    actor_msg->width = 0;

    PointCloud::Ptr table_msg (new PointCloud);
    table_msg->header.frame_id = "velodyne";
    table_msg->height = msg->height;
    table_msg->width = 0;

    PointCloud::Ptr chair_msg (new PointCloud);
    chair_msg->header.frame_id = "velodyne";
    chair_msg->height = msg->height;
    chair_msg->width = 0;
    
    // reconstruct vehicle quadtree:
    this->vehicle_quadtree = boost::make_shared<QuadTree>(this->building_box);
    for (auto act: this->actors){
        auto actor_pos = act->WorldPose().Pos();
        auto min = ignition::math::Vector3d(actor_pos.X() - 0.25, actor_pos.Y() - 0.25, 0);
        auto max = ignition::math::Vector3d(actor_pos.X() + 0.25, actor_pos.Y() + 0.25, 0);
        auto box = ignition::math::Box(min,max);
        auto new_node = QTData(box, act, entity_type);
        this->vehicle_quadtree->Insert(new_node);
    }

    for (auto pt : msg->points){

        auto point = this->sensor_pose.CoordPositionAdd(ignition::math::Vector3d(pt.x, pt.y, pt.z));       
        //auto point = ignition::math::Vector3d(pt.x, pt.y, pt.z);
        //point+=this->sensor_pose.Pos();
        
   
        std::vector<gazebo::physics::EntityPtr> near_vehicles;
        std::vector<gazebo::physics::EntityPtr> near_objects;


        double resolution = 0.01;
        auto min = ignition::math::Vector3d(point.X() - resolution, point.Y() - resolution, 0);
        auto max = ignition::math::Vector3d(point.X() + resolution, point.Y() + resolution, 0);
        auto query_range = ignition::math::Box(min,max);

        std::vector<QTData> query_objects = this->static_quadtree->QueryRange(query_range);
        for (auto n: query_objects){
            if (n.type == entity_type){
                near_objects.push_back(boost::static_pointer_cast<gazebo::physics::Entity>(n.data));
                
            }
        }


        std::vector<QTData> query_vehicles = this->vehicle_quadtree->QueryRange(query_range);
        for (auto n: query_vehicles){
            if (n.type == entity_type){
                near_vehicles.push_back(boost::static_pointer_cast<gazebo::physics::Entity>(n.data));
            }
        }

        //std::cout << near_objects.size() << std::endl;
        if (near_objects.size() == 0 && near_vehicles.size() == 0){
            //std::cout << "ground" << std::endl;
            ground_msg->points.push_back(pt);
            ground_msg->width++;
            //std::cout << point.Z() << std::endl;
        } else {

            std::string closest_name = "ground_plane"; 
            double min_dist = std::abs(point.Z());
            


            for (auto n: near_objects){
                
                auto dist = utilities::dist_to_box(point, n->BoundingBox());
                if (n->GetName().substr(0,4) == "Wall"){
                    std::cout << dist << std::endl;
                }
                if (dist <= min_dist){
                    min_dist = dist;
                    closest_name == n->GetName();
                    if (closest_name.substr(0,4) != "Wall"){
                        closest_name = n->GetParent()->GetName();
                    }
                }

                /*
                if ((n->GetName()).substr(0,4) == "Wall"){
                    wall_msg->points.push_back(pt);
                    wall_msg->width++;
                } else if ((n->GetParent()->GetName()).substr(0,5) == "table"){
                    table_msg->points.push_back(pt);
                    table_msg->width++;
                } else if ((n->GetParent()->GetName()).substr(0,5) == "chair"){
                    chair_msg->points.push_back(pt);
                    chair_msg->width++;
                } 
                */
                
            }

            if (closest_name.substr(0,4) == "Wall"){
                wall_msg->points.push_back(pt);
                wall_msg->width++;
            } else if (closest_name.substr(0,5) == "table"){
                table_msg->points.push_back(pt);
                table_msg->width++;
            } else if (closest_name.substr(0,5) == "chair"){
                chair_msg->points.push_back(pt);
                chair_msg->width++;
            } else {

                ground_msg->points.push_back(pt);
                ground_msg->width++;
            }

            
            
            for (auto n: near_vehicles){
                actor_msg->points.push_back(pt);
                actor_msg->width++;
            }
           
        }
        //std::printf ("\t(%f, %f, %f)\n", point.X(), point.Y(), point.Z());
    }

    pcl_conversions::toPCL(ros::Time::now(), ground_msg->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), wall_msg->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), actor_msg->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), table_msg->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), chair_msg->header.stamp);
   

    this->ground_pub.publish(ground_msg);
    this->wall_pub.publish(wall_msg);
    this->actor_pub.publish(actor_msg);
    this->table_pub.publish(table_msg);
    this->chair_pub.publish(chair_msg);
}


void LidarListener::ReadSDF(){
    if (this->sdf->HasElement("building_name")){
        this->building_name =this->sdf->GetElement("building_name")->Get<std::string>();
    }
    if (this->sdf->HasElement("robot_name")){
        this->robot_name = this->sdf->GetElement("robot_name")->Get<std::string>();
    }
}
