#include "puppeteer.hh"
#include "utilities.hh"

GZ_REGISTER_WORLD_PLUGIN(Puppeteer);

//PUPPETEER CLASS

void Puppeteer::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){


    this->world = _world;
    this->sdf = _sdf;
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&Puppeteer::OnUpdate, this, std::placeholders::_1));

    this->ReadSDF();

    this->ReadParams();

    

    auto building = this->world->ModelByName(this->building_name);

    this->building_box = building->BoundingBox();
    this->building_box.Min().X()-=1;
    this->building_box.Min().Y()-=1;
    this->building_box.Max().X()+=1;
    this->building_box.Max().Y()+=1;
    this->static_quadtree = boost::make_shared<QuadTree>(this->building_box);
    this->vehicle_quadtree = boost::make_shared<QuadTree>(this->building_box);
    
    
    this->fields.push_back(boost::make_shared<FlowField>(ignition::math::Vector3d(building_box.Min().X(),building_box.Max().Y(),0), building_box.Max().X() - building_box.Min().X(), building_box.Max().Y() - building_box.Min().Y(), 0.2));
    
    for (unsigned int i = 0; i < world->ModelCount(); ++i) {
        auto model = world->ModelByIndex(i);
        auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

        if (act){
            
            auto new_vehicle = this->CreateVehicle(act);
            this->vehicles.push_back(new_vehicle);
            auto min = ignition::math::Vector3d(new_vehicle->GetPose().Pos().X() - 0.4, new_vehicle->GetPose().Pos().Y() - 0.4, 0);
            auto max = ignition::math::Vector3d(new_vehicle->GetPose().Pos().X() + 0.4, new_vehicle->GetPose().Pos().Y() + 0.4, 0);
            auto box = ignition::math::Box(min,max);
            auto new_node = QTData(box, new_vehicle, vehicle_type);
            this->vehicle_quadtree->Insert(new_node);
            continue;
        } 

        
        if (model->GetName() != "ground_plane"){
            auto links = model->GetLinks();
            for (gazebo::physics::LinkPtr link: links){
                std::vector<gazebo::physics::CollisionPtr> collision_boxes = link->GetCollisions();
                for (gazebo::physics::CollisionPtr collision_box: collision_boxes){
                    this->collision_entities.push_back(collision_box);
                    auto box = collision_box->BoundingBox();
                    box.Max().Z() = 0;
                    box.Min().Z() = 0;
                    auto new_node = QTData(box, collision_box, entity_type);
                    this->static_quadtree->Insert(new_node);
                }
                    
            }
        }
        

       /*
        if (model->GetName() == this->building_name){
            auto links = model->GetLinks();
            for (gazebo::physics::LinkPtr link: links){
                std::vector<gazebo::physics::CollisionPtr> collision_boxes = link->GetCollisions();
                for (gazebo::physics::CollisionPtr collision_box: collision_boxes){
                    this->collision_entities.push_back(collision_box); //TODO: check if this is correct (maybe do dynamic pointer cast )
                    auto new_node = QTData(collision_box->BoundingBox(), collision_box, entity_type);
                    this->static_quadtree->Insert(new_node);
                }
                    
            }
        } else if (model->GetName() != "ground_plane"){
            this->collision_entities.push_back(model);
            auto box = model->BoundingBox();
            box.Min().Z() = 0;
            box.Max().Z() = 0;
            auto new_node = QTData(box, model, entity_type);
            this->static_quadtree->Insert(new_node);
        } 
        */
       
    }

    this->fields[0]->TargetInit(this->collision_entities, ignition::math::Vector3d(5,5,0));

    std::cout << "LOADED ALL VEHICLES\n";

    if (this->lidar_listener){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "LidarListener");
        
        this->sub = this->nh.subscribe<PointCloud>("velodyne_points", 1, &Puppeteer::Callback, this);
        this->ground_pub = nh.advertise<PointCloud>("ground_points", 1);
        this->wall_pub = nh.advertise<PointCloud>("wall_points", 1);
        this->moving_actor_pub = nh.advertise<PointCloud>("moving_actor_points", 1);
        this->still_actor_pub = nh.advertise<PointCloud>("still_actor_points", 1);
        this->table_pub = nh.advertise<PointCloud>("table_points", 1);
        this->chair_pub = nh.advertise<PointCloud>("chair_points", 1);
        ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
        ros::Rate r = (ros::Rate) this->update_freq;
        std::cout << "Advertising Lidar Points\n";
        spinner.start();

    }

}

void Puppeteer::OnUpdate(const gazebo::common::UpdateInfo &_info){
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
                for (auto vehicle: this->follower_queue){
                    vehicle->LoadLeader(this->robot);
                }
                this->robot_links = this->robot->GetLinks();
                std::cout << "ADDED ROBOT: " << this->robot->GetName() << std::endl;
            }
        }
    }

    if (this->robot != nullptr){
        this->sensor_pose = this->robot_links[0]->WorldPose();
        this->sensor_pose.Pos().Z() += 0.5767;
    }


    // reconstruct vehicle_quad tree

    this->vehicle_quadtree = boost::make_shared<QuadTree>(this->building_box);

    for (auto vehicle: this->vehicles){
        auto min = ignition::math::Vector3d(vehicle->GetPose().Pos().X() - 0.4, vehicle->GetPose().Pos().Y() - 0.4, 0);
        auto max = ignition::math::Vector3d(vehicle->GetPose().Pos().X() + 0.4, vehicle->GetPose().Pos().Y() + 0.4, 0);
        auto box = ignition::math::Box(min,max);
        auto new_node = QTData(box, vehicle, vehicle_type);
        this->vehicle_quadtree->Insert(new_node);
    }

    for (auto vehicle: this->vehicles){
        // query quad tree
        std::vector<boost::shared_ptr<Vehicle>> near_vehicles;

        std::vector<gazebo::physics::EntityPtr> near_objects;

        auto vehicle_pos = vehicle->GetPose();

        auto min = ignition::math::Vector3d(vehicle_pos.Pos().X() - 2, vehicle_pos.Pos().Y() - 2, 0);
        auto max = ignition::math::Vector3d(vehicle_pos.Pos().X() + 2, vehicle_pos.Pos().Y() + 2, 0);
        auto query_range = ignition::math::Box(min,max);

        std::vector<QTData> query_objects = this->static_quadtree->QueryRange(query_range);
        for (auto n: query_objects){
            if (n.type == entity_type){
                near_objects.push_back(boost::static_pointer_cast<gazebo::physics::Entity>(n.data));
            }
            
        }

        if (this->robot != nullptr && (vehicle_pos.Pos() - this->robot->WorldPose().Pos()).Length()<2){
            near_objects.push_back(this->robot);
        }

        std::vector<QTData> query_vehicles = this->vehicle_quadtree->QueryRange(query_range);
        for (auto n: query_vehicles){
            if (n.type == vehicle_type){
                near_vehicles.push_back(boost::static_pointer_cast<Vehicle>(n.data));
            }
        }

        vehicle->OnUpdate(_info, dt, near_vehicles, near_objects);
    }
}

void Puppeteer::ReadSDF(){
    if (this->sdf->HasElement("building_name")){
        this->building_name =this->sdf->GetElement("building_name")->Get<std::string>();
    }
    
    if (this->sdf->HasElement("robot_name")){
        this->robot_name = this->sdf->GetElement("robot_name")->Get<std::string>();
        //std::cout << this->robot_name << std::endl;
    }

    if (this->sdf->HasElement("lidar_listener")){
        this->lidar_listener = this->sdf->GetElement("lidar_listener")->Get<bool>();
    }
}

boost::shared_ptr<Vehicle> Puppeteer::CreateVehicle(gazebo::physics::ActorPtr actor){

    boost::shared_ptr<Vehicle> res;
   
    auto sdf = actor->GetSDF();

    std::map<std::string, std::string> actor_info; 

    auto attribute = sdf->GetElement("plugin");
        
	while (attribute){
		actor_info[attribute->GetAttribute("name")->GetAsString()] = attribute->GetAttribute("filename")->GetAsString();
		attribute = attribute->GetNextElement("plugin");
	}

    double max_speed = 1;

    if (actor_info.find("max_speed") != actor_info.end()){
        try{
            max_speed = std::stod(actor_info["max_speed"]);
        } catch (std::exception& e){
            std::cout << "Error converting max_speed argument to double" << std::endl;
        }
    }

    if (actor_info.find("vehicle_type")!=actor_info.end()){
        if (actor_info["vehicle_type"] == "wanderer"){

            res = boost::make_shared<Wanderer>(actor, this->vehicle_params["mass"], this->vehicle_params["max_force"], max_speed, actor->WorldPose(), ignition::math::Vector3d(0,0,0), this->collision_entities);
        
        } else if (actor_info["vehicle_type"] == "random_walker"){
            res = boost::make_shared<RandomWalker>(actor, this->vehicle_params["mass"], this->vehicle_params["max_force"], max_speed, actor->WorldPose(), ignition::math::Vector3d(0,0,0), this->collision_entities);
            
        } else if (actor_info["vehicle_type"] == "boid"){
            auto random_vel = ignition::math::Vector3d(ignition::math::Rand::DblUniform(-1,1),ignition::math::Rand::DblUniform(-1,1),0);
            random_vel.Normalize(); 
            random_vel*=2;
            res = boost::make_shared<Boid>(actor, this->vehicle_params["mass"], this->vehicle_params["max_force"], max_speed, actor->WorldPose(), random_vel, this->collision_entities, this->boid_params["alignement"], this->boid_params["cohesion"], this->boid_params["separation"], this->boid_params["FOV_angle"], this->boid_params["FOV_radius"]); // read in as params 
            
        } else if (actor_info["vehicle_type"] == "stander"){
            
            double standing_duration = 5;
            double walking_duration = 5;

            if (actor_info.find("standing_duration") != actor_info.end()){
                try{
                    standing_duration = std::stod(actor_info["standing_duration"]);
                } catch (std::exception& e){
                    std::cout << "Error converting standing duration argument to double" << std::endl;
                }
            }

            if (actor_info.find("walking_duration") != actor_info.end()){
                try{
                    walking_duration = std::stod(actor_info["walking_duration"]);
                } catch (std::exception& e){
                    std::cout << "Error converting walking duration argument to double" << std::endl;
                }
            }

            res = boost::make_shared<Stander>(actor, 1, 10, max_speed, actor->WorldPose(), ignition::math::Vector3d(0,0,0), this->collision_entities, standing_duration, walking_duration); // read in as params 
            
        } else if (actor_info["vehicle_type"] == "sitter"){

            std::string chair = "";

            if (actor_info.find("chair") != actor_info.end()){
                chair = actor_info["chair"];
            }

            res = boost::make_shared<Sitter>(actor, chair, this->collision_entities, actor->WorldPose().Pos().Z());

        } else if (actor_info["vehicle_type"] == "follower"){

            std::string leader_name = "";
            
            if (actor_info.find("leader") != actor_info.end()){
                leader_name = actor_info["leader"];
               
                res = boost::make_shared<Follower>(actor, 1, 10, max_speed, actor->WorldPose(), ignition::math::Vector3d(0,0,0), this->collision_entities, leader_name); // read in as params 
                this->follower_queue.push_back(boost::dynamic_pointer_cast<Follower>(res));
            } else{
                std::cout << "leader name not found\n";
            }
        } else if (actor_info["vehicle_type"] == "flow_follower"){

            res = boost::make_shared<FlowFollower>(actor, this->vehicle_params["mass"], this->vehicle_params["max_force"], max_speed, actor->WorldPose(), ignition::math::Vector3d(0,0,0), this->collision_entities, this->fields);
            
        } else {
            std::cout << "INVALID VEHICLE TYPE\n";
        }
    }
    

    return res;
}

void Puppeteer::ReadParams(){
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "Puppeteer");
    ros::NodeHandle nh;

    if (!nh.getParam("common_vehicle_params", this->vehicle_params)){
        ROS_ERROR("ERROR READING COMMON VEHICLE PARAMS");
        vehicle_params["mass"] = 1;
        vehicle_params["max_force"] = 10;
        vehicle_params["slowing_distance"] =  2;
        vehicle_params["arrival_distance"] = 0.5;
        vehicle_params["obstacle_margin"] = 0.6;
    }

    if (!nh.getParam("common_boid_params", this->boid_params)){
        ROS_ERROR("ERROR READING COMMON BOID PARAMS");
        boid_params["alignement"] =  0.1;
        boid_params["cohesion"] =  0.01;
        boid_params["separation"] =  2;
        boid_params["FOV_angle"] =  4;
        boid_params["FOV_radius"] =  3;
    }
}

void Puppeteer::Callback(const PointCloud::ConstPtr& msg){

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
    
    PointCloud::Ptr moving_actor_msg (new PointCloud);
    moving_actor_msg->header.frame_id = "velodyne";
    moving_actor_msg->height = msg->height;
    moving_actor_msg->width = 0;

    PointCloud::Ptr still_actor_msg (new PointCloud);
    still_actor_msg->header.frame_id = "velodyne";
    still_actor_msg->height = msg->height;
    still_actor_msg->width = 0;

    PointCloud::Ptr table_msg (new PointCloud);
    table_msg->header.frame_id = "velodyne";
    table_msg->height = msg->height;
    table_msg->width = 0;

    PointCloud::Ptr chair_msg (new PointCloud);
    chair_msg->header.frame_id = "velodyne";
    chair_msg->height = msg->height;
    chair_msg->width = 0;

    this->vehicle_quadtree2 = boost::make_shared<QuadTree>(this->building_box);

    for (auto vehicle: this->vehicles){
        auto min = ignition::math::Vector3d(vehicle->GetPose().Pos().X() - 0.3, vehicle->GetPose().Pos().Y() - 0.3, 0);
        auto max = ignition::math::Vector3d(vehicle->GetPose().Pos().X() + 0.3, vehicle->GetPose().Pos().Y() + 0.3, 0);
        auto box = ignition::math::Box(min,max);
        auto new_node = QTData(box, vehicle, vehicle_type);
        this->vehicle_quadtree2->Insert(new_node);
    }
    

    for (auto pt : msg->points){

        auto point = this->sensor_pose.CoordPositionAdd(ignition::math::Vector3d(pt.x, pt.y, pt.z));       
   
        std::vector<boost::shared_ptr<Vehicle>> near_vehicles;
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

        std::vector<QTData> query_vehicles = this->vehicle_quadtree2->QueryRange(query_range);
        for (auto n: query_vehicles){
            if (n.type == vehicle_type){
                near_vehicles.push_back(boost::static_pointer_cast<Vehicle>(n.data));
            }
        }

   
        if (near_objects.size() == 0 && near_vehicles.size() == 0){
            
            ground_msg->points.push_back(pt);
            ground_msg->width++;
        
        } else {
            
            std::string closest_name = "ground_plane";
            double min_dist = point.Z();
            
            for (auto n: near_objects){
                
                auto dist = utilities::dist_to_box(point, n->BoundingBox());

                if (dist <= min_dist){
                    min_dist = dist;
                    closest_name = n->GetParent()->GetParent()->GetName();
                }
            }
 
            if (closest_name == this->building_name){
                wall_msg->points.push_back(pt);
                wall_msg->width++;
            } else if (closest_name.substr(0,5) == "table"){
                table_msg->points.push_back(pt);
                table_msg->width++;
            } else if (closest_name.substr(0,5) == "chair" && near_vehicles.size() == 0){
                chair_msg->points.push_back(pt);
                chair_msg->width++;
            } else if (near_vehicles.size() == 0) {
                ground_msg->points.push_back(pt);
                ground_msg->width++;
            }
  

            
            
            for (auto n: near_vehicles){
                
                if (point.Z() <=0){
                    ground_msg->points.push_back(pt);
                    ground_msg->width++;
                } else{
                  
                    if (n->IsStill()){
                        still_actor_msg->points.push_back(pt);
                        still_actor_msg->width++;
                    } else{
                        moving_actor_msg->points.push_back(pt);
                        moving_actor_msg->width++;
                    }
                    
                }
            }

           
        }
        
    }

    pcl_conversions::toPCL(ros::Time::now(), ground_msg->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), wall_msg->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), moving_actor_msg->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), still_actor_msg->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), table_msg->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), chair_msg->header.stamp);
   

    this->ground_pub.publish(ground_msg);
    this->wall_pub.publish(wall_msg);
    this->moving_actor_pub.publish(moving_actor_msg);
    this->still_actor_pub.publish(still_actor_msg);
    this->table_pub.publish(table_msg);
    this->chair_pub.publish(chair_msg);
}