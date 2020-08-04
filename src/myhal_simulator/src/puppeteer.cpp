#include "puppeteer.hh"
#include "utilities.hh"
#include "parse_tour.hh"
#include <ros/forwards.h>

GZ_REGISTER_WORLD_PLUGIN(Puppeteer);

//PUPPETEER CLASS

void Puppeteer::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){
    //note: world name is default

    this->world = _world;
    this->sdf = _sdf;
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&Puppeteer::OnUpdate, this, std::placeholders::_1));

    this->user_name = "default";

    if (const char * user = std::getenv("USER")){
        this->user_name = user;
    } 

    this->ReadSDF();

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "Puppeteer");

    this->ReadParams();

    this->path_pub = this->nh.advertise<geometry_msgs::PoseStamped>("optimal_path",1000);

    auto building = this->world->ModelByName(this->building_name);

    this->building_box = building->BoundingBox();
    this->building_box.Min().X()-=1;
    this->building_box.Min().Y()-=1;
    this->building_box.Max().X()+=1;
    this->building_box.Max().Y()+=1;
    this->static_quadtree = boost::make_shared<QuadTree>(this->building_box);
    this->vehicle_quadtree = boost::make_shared<QuadTree>(this->building_box);
    this->costmap = boost::make_shared<Costmap>(this->building_box, 0.2);
    
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

        if (model->GetName().substr(0,3) == "CAM"){
            this->cams.push_back(this->CreateCamera(model));
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

                    if (collision_box->BoundingBox().Min().Z() < 1.5 && collision_box->BoundingBox().Max().Z() > 10e-2){
                        box.Min().X()-=0.2;
                        box.Min().Y()-=0.2;
                        box.Max().X()+=0.2;
                        box.Max().Y()+=0.2;
                        this->costmap->AddObject(box);
                    }
                }
            }
        }
    }

    if (this->tour_name != ""){
        TourParser parser = TourParser(this->tour_name);
        auto route = parser.GetRoute();
        route.insert(route.begin(), ignition::math::Pose3d(ignition::math::Vector3d(0,0,0), ignition::math::Quaterniond(0,0,0,1)));

        for (int i =0; i< route.size()-1; i++){
            auto start = route[i];
            auto end = route[i+1];
            std::vector<ignition::math::Vector3d> path;
            this->costmap->ThetaStar(start.Pos(), end.Pos(), path);
            this->paths.push_back(path);
        }
    }

    std::cout << "LOADED ALL VEHICLES\n";

    std::cout << "COMMAND: " << this->launch_command << std::endl;
    std::system(this->launch_command.c_str());

    if (this->viz_gaz){
        this->global_plan_sub = this->nh.subscribe("/move_base/NavfnROS/plan", 1, &Puppeteer::GlobalPlanCallback, this);
        this->local_plan_sub = this->nh.subscribe("/move_base/TrajectoryPlannerROS/local_plan", 1, &Puppeteer::LocalPlanCallback, this);
        this->nav_goal_sub = this->nh.subscribe("/move_base/goal", 1, &Puppeteer::NavGoalCallback, this);
        
        //ros::AsyncSpinner spinner(4); // Use 4 threads
        //spinner.start();
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
        
        if (this->tour_name != ""){
            double i = 0;
            for (auto path: this->paths){
                for (auto pose: path){
                    geometry_msgs::PoseStamped msg;
                    msg.pose.position.x = pose.X();
                    msg.pose.position.y = pose.Y();
                    msg.pose.position.z = pose.Z();
                    msg.header.stamp = ros::Time(i);
                    path_pub.publish(msg);
                }
                i++;
            }
        }

        return;
    }

    if (this->robot != nullptr){
        this->robot_traj.push_back(this->robot->WorldPose().Pos());
        this->robot_traj.back().Z() = 0;
        for (auto cam: this->cams){
            cam->OnUpdate(dt, this->robot_traj);
        }
    }


    this->vehicle_quadtree = boost::make_shared<QuadTree>(this->building_box);

    for (auto vehicle: this->vehicles){
        auto min = ignition::math::Vector3d(vehicle->GetPose().Pos().X() - 0.4, vehicle->GetPose().Pos().Y() - 0.4, 0);
        auto max = ignition::math::Vector3d(vehicle->GetPose().Pos().X() + 0.4, vehicle->GetPose().Pos().Y() + 0.4, 0);
        auto box = ignition::math::Box(min,max);
        auto new_node = QTData(box, vehicle, vehicle_type);
        this->vehicle_quadtree->Insert(new_node);
        
    }

    for (auto vehicle: this->vehicles){
    
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


    if (this->old_global_ind != this->new_global_ind){
        std::string to_remove = "global_plan_" + std::to_string(this->old_global_ind);
        if (this->world->EntityByName(to_remove)){
            this->world->RemoveModel(to_remove);
            std::cout << utilities::color_text("Removing global plan: " + to_remove, GREEN) << std::endl;
        }
        std::string to_add = "global_plan_" + std::to_string(this->new_global_ind);
        this->AddPathMarkers(to_add, this->global_plan, ignition::math::Vector4d(0,1,0,1));
        std::cout << utilities::color_text("Adding global plan: " +  to_add, GREEN) << std::endl;
        this->old_global_ind = this->new_global_ind;
        std::cout << utilities::color_text("Model Count: " + std::to_string(this->world->ModelCount()), BLUE) << std::endl;
    }

    if (this->old_local_ind != this->new_local_ind){
        std::string to_remove = "local_plan_" + std::to_string(this->old_local_ind);
        if (this->world->EntityByName(to_remove)){
            this->world->RemoveModel(to_remove);
            std::cout << utilities::color_text("Removing local plan: " + to_remove, GREEN) << std::endl;
        }
        std::string to_add = "local_plan_" + std::to_string(this->new_local_ind);
        this->AddPathMarkers(to_add, this->local_plan, ignition::math::Vector4d(0,0,1,1));
        std::cout << utilities::color_text("Adding local plan: " + to_remove, GREEN) << std::endl;
        this->old_local_ind = this->new_local_ind;
        std::cout << utilities::color_text("Model Count: " + std::to_string(this->world->ModelCount()), BLUE) << std::endl;
    }

    if (this->old_nav_ind != this->new_nav_ind){
        std::string name = "nav_goal";
        if (this->old_nav_ind == 0){
            this->AddGoalMarker(name, this->nav_goal, ignition::math::Vector4d(1,0,0,1));
        } else{
            auto goal = this->world->EntityByName(name);
            auto p = this->nav_goal->goal.target_pose.pose.position;
            auto pos = ignition::math::Vector3d(p.x, p.y, p.z);
            goal->SetWorldPose(ignition::math::Pose3d(pos, ignition::math::Quaterniond(0,0,0,1)));
        }
        this->old_nav_ind = this->new_nav_ind;
    }

    if (this->viz_gaz){
        ros::spinOnce();
    }

}

void Puppeteer::ReadSDF(){

    if (this->sdf->HasElement("building_name")){
        this->building_name =this->sdf->GetElement("building_name")->Get<std::string>();
    }
    
    if (this->sdf->HasElement("robot_name")){
        this->robot_name = this->sdf->GetElement("robot_name")->Get<std::string>();
    } else{
        this->robot_name = "";
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

            res = boost::make_shared<Stander>(actor, 1, 10, max_speed, actor->WorldPose(), ignition::math::Vector3d(0,0,0), this->collision_entities, standing_duration, walking_duration, this->vehicle_params["start_mode"]); // read in as params 
            
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
                
               
                res = boost::make_shared<Follower>(actor, 1, 10, max_speed, actor->WorldPose(), ignition::math::Vector3d(0,0,0), this->collision_entities, leader_name, (bool) this->vehicle_params["blocking"]); // read in as params 
                this->follower_queue.push_back(boost::dynamic_pointer_cast<Follower>(res));
            } else{
                std::cout << "leader name not found\n";
            }
        } else if (actor_info["vehicle_type"] == "path_follower"){

            res = boost::make_shared<PathFollower>(actor, this->vehicle_params["mass"], this->vehicle_params["max_force"], max_speed, actor->WorldPose(), ignition::math::Vector3d(0,0,0), this->collision_entities, this->costmap);
            
        } else {
            std::cout << "INVALID VEHICLE TYPE\n";
        }
    }
    

    return res;
}

void Puppeteer::ReadParams(){

    if (!nh.getParam("common_vehicle_params", this->vehicle_params)){
        ROS_ERROR("ERROR READING COMMON VEHICLE PARAMS");
        vehicle_params["mass"] = 1;
        vehicle_params["max_force"] = 10;
        vehicle_params["slowing_distance"] =  2;
        vehicle_params["arrival_distance"] = 0.5;
        vehicle_params["obstacle_margin"] = 0.6;
        vehicle_params["blocking"] = 0;
        vehicle_params["start_mode"] = 2;
    }

    if (!nh.getParam("common_boid_params", this->boid_params)){
        ROS_ERROR("ERROR READING COMMON BOID PARAMS");
        boid_params["alignement"] =  0.1;
        boid_params["cohesion"] =  0.01;
        boid_params["separation"] =  2;
        boid_params["FOV_angle"] =  4;
        boid_params["FOV_radius"] =  3;
    }
    
    if (!nh.getParam("gt_class", this->gt_class)){
        std::cout << "ERROR READING gt_class\n";
        this->gt_class = false;
    }

    if (!nh.getParam("filter_status", this->filter_status)){
        std::cout << "ERROR READING filter_status\n";
        this->filter_status = false;
    }

    if (!nh.getParam("gmapping_status", this->gmapping_status)){
        std::cout << "ERROR READING gmapping_status\n";
        this->gmapping_status = false;
    }

    if (!nh.getParam("viz_gaz", this->viz_gaz)){
        std::cout << "ERROR READING viz_gaz\n";
        this->viz_gaz = false;
    }
    //roslaunch jackal_velodyne p2.launch filter:=$FILTER mapping:=$MAPPING gt_classify:=$GTCLASS 
    if (this->filter_status){
        this->launch_command += " filter:=true ";
    } else {
        this->launch_command += " filter:=false ";
    }
    if (this->gt_class){
        this->launch_command += " gt_classify:=true ";
    } else{
        this->launch_command += " gt_classify:=false ";
    }
    if (this->gmapping_status){
        this->launch_command += " mapping:=true &";
    } else{
        this->launch_command += " mapping:=false &";
    }

    std::cout << "COMMAND: " << this->launch_command << std::endl;

    if (!nh.getParam("tour_name", this->tour_name)){
        std::cout << "ERROR READING TOUR NAME\n";
        this->tour_name = "";
        return;
    }

}

SmartCamPtr Puppeteer::CreateCamera(gazebo::physics::ModelPtr model){
    auto tokens = utilities::split(model->GetName(), '_');
    SmartCamPtr new_cam;
    if (tokens[1] == "0"){
        // sentry
        new_cam = boost::make_shared<Sentry>(model,model->WorldPose().Pos()); 
    } else if (tokens[1] == "1"){
        // hoverer
        double T = std::stod(tokens[2]);
        new_cam = boost::make_shared<Hoverer>(model, model->WorldPose().Pos(), T); 
    } else if (tokens[1] == "2"){
        // path follower
        double dist = std::stod(tokens[3]);
        new_cam = boost::make_shared<Stalker>(model, model->WorldPose().Pos(), dist); 
    }
    return new_cam;
}

void Puppeteer::GlobalPlanCallback(const nav_msgs::Path::ConstPtr& path){
    if (path->poses.size() > 0){
        std::cout << utilities::color_text("Global plan recieved by simulator", YELLOW) << std::endl;
        this->global_plan = path;
        this->new_global_ind++;
    }
}

void Puppeteer::LocalPlanCallback(const nav_msgs::Path::ConstPtr& path){
    if (path->poses.size() > 0){
        std::cout << utilities::color_text("Local plan recieved by simulator", YELLOW) << std::endl;
        this->local_plan = path;
        this->new_local_ind++;
    }
}
        
void Puppeteer::NavGoalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal){
    this->nav_goal = goal;
    this->new_nav_ind++;
}

void Puppeteer::AddPathMarkers(std::string name, const nav_msgs::Path::ConstPtr& plan, ignition::math::Vector4d color){

    boost::shared_ptr<sdf::SDF> sdf = boost::make_shared<sdf::SDF>();
    sdf->SetFromString(
       "<sdf version ='1.6'>\
          <model name ='path'>\
          </model>\
        </sdf>");

    auto model = sdf->Root()->GetElement("model");
    model->GetElement("static")->Set(true);
    model->GetAttribute("name")->SetFromString(name);

    int i = 0;
    for (auto pose: plan->poses){
        auto p = pose.pose.position;
        auto pos = ignition::math::Vector3d(p.x, p.y, p.z);
        auto link = model->AddElement("link");
        link->GetElement("pose")->Set(pos);
        link->GetAttribute("name")->SetFromString("l_" + name + std::to_string(i));
        auto cylinder = link->GetElement("visual")->GetElement("geometry")->GetElement("cylinder");
        cylinder->GetElement("radius")->Set(0.03);
        cylinder->GetElement("length")->Set(0.001);
        auto mat = link->GetElement("visual")->GetElement("material");
        mat->GetElement("ambient")->Set(color);
        mat->GetElement("diffuse")->Set(color);
        mat->GetElement("specular")->Set(color);
        mat->GetElement("emissive")->Set(color);
        i++;
    }

    this->world->InsertModelSDF(*sdf);
}


void Puppeteer::AddGoalMarker(std::string name, const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal, ignition::math::Vector4d color){

    auto p = goal->goal.target_pose.pose.position;
    auto pos = ignition::math::Vector3d(p.x, p.y, p.z);
        
    boost::shared_ptr<sdf::SDF> sdf = boost::make_shared<sdf::SDF>();
    sdf->SetFromString(
       "<sdf version ='1.6'>\
          <model name ='path'>\
          </model>\
        </sdf>");

    auto model = sdf->Root()->GetElement("model");
    model->GetElement("static")->Set(true);
    model->GetAttribute("name")->SetFromString(name);
    model->GetElement("pose")->Set(pos);

    auto link1 = model->AddElement("link");
    link1->GetAttribute("name")->SetFromString("l_1" + name);
    auto box1 = link1->GetElement("visual")->GetElement("geometry")->GetElement("box");
    box1->GetElement("size")->Set(ignition::math::Vector3d(0.5, 0.05, 0.001));
    auto mat1 = link1->GetElement("visual")->GetElement("material");
    mat1->GetElement("ambient")->Set(color);
    mat1->GetElement("diffuse")->Set(color);
    mat1->GetElement("specular")->Set(color);
    mat1->GetElement("emissive")->Set(color);

    auto link2 = model->AddElement("link");
    link2->GetAttribute("name")->SetFromString("l_2" + name);
    auto box2 = link2->GetElement("visual")->GetElement("geometry")->GetElement("box");
    box2->GetElement("size")->Set(ignition::math::Vector3d(0.05, 0.5, 0.001));
    auto mat2 = link2->GetElement("visual")->GetElement("material");
    mat2->GetElement("ambient")->Set(color);
    mat2->GetElement("diffuse")->Set(color);
    mat2->GetElement("specular")->Set(color);
    mat2->GetElement("emissive")->Set(color);

    this->world->InsertModelSDF(*sdf);
}
