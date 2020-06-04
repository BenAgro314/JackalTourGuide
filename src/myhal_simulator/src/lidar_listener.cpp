#include "lidar_listener.hh"


GZ_REGISTER_WORLD_PLUGIN(LidarListener);

//void Callback(const PointCloud::ConstPtr& msg);

void LidarListener::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){
    this->world = _world;
    this->sdf = _sdf;
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&LidarListener::OnUpdate, this, std::placeholders::_1));

    this->ReadSDF();

    auto building = this->world->ModelByName(this->building_name);

    auto building_box = building->BoundingBox();
    building_box.Min().X()-=1;
    building_box.Min().Y()-=1;
    building_box.Max().X()+=1;
    building_box.Max().Y()+=1;
    this->static_quadtree = boost::make_shared<QuadTree>(building_box);
    this->vehicle_quadtree = boost::make_shared<QuadTree>(building_box);

    

    for (unsigned int i = 0; i < world->ModelCount(); ++i) {
        auto model = world->ModelByIndex(i);
        auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

        if (act){
            this->actors.push_back(act);
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
            this->model_collisions.push_back(model);
            auto box = model->BoundingBox();
            box.Min().Z() = 0;
            box.Max().Z() = 0;
            auto new_node = QTData(box, model, entity_type);
            this->static_quadtree->Insert(new_node);
        } 
        
    }

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "LidarListener");
    
    this->sub = this->nh.subscribe<PointCloud>("velodyne_points", 1, &LidarListener::Callback, this);
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    ros::Rate r= 10;
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
                for (auto link: this->robot_links){
                    std::cout << link->GetName() << std::endl;
                }
            }
        }
    } 

    if (this->robot != nullptr){
        this->sensor_pose = this->robot_links[0]->WorldPose();
        this->sensor_pose.Pos().Z() += 0.539;
        //std::printf("%f %f %f\n", sensor_pose.Pos().X(), sensor_pose.Pos().Y(), sensor_pose.Pos().Z());
    }

    //std::cout << "hi" << std::endl;

   
}

void LidarListener::ReadSDF(){
    if (this->sdf->HasElement("building_name")){
        this->building_name =this->sdf->GetElement("building_name")->Get<std::string>();
    }
    if (this->sdf->HasElement("robot_name")){
        this->robot_name = this->sdf->GetElement("robot_name")->Get<std::string>();
    }
}

void LidarListener::Callback(const PointCloud::ConstPtr& msg){

    if (this->robot == nullptr){
        return;
    }

    //std::printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    // const pcl::PointXYZ& pt : msg->point
    //double min_z = 10000;
    for (auto pt : msg->points){
        auto point = ignition::math::Vector3d(pt.x, pt.y, pt.z);
        point+=this->sensor_pose.Pos();
        min_z = std::min(min_z, point.Z());
        //std::printf ("\t(%f, %f, %f)\n", point.X(), point.Y(), point.Z());
    }
    //std::cout << min_z << std::endl;
    
    
}