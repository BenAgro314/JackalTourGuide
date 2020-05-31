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

    for (unsigned int i = 0; i < world->ModelCount(); ++i) {
        auto model = world->ModelByIndex(i);
        //std::cout << model->GetName() << std::endl;

        auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

        if (act){
            //std::string prefix = "";
            //std::cout << act->GetSDF()->ToString(prefix) << std::endl;
            auto new_vehicle = this->CreateVehicle(act);
            this->vehicles.push_back(new_vehicle);
            // check this bounding box
            auto min = ignition::math::Vector3d(new_vehicle->GetPose().Pos().X() - 0.4, new_vehicle->GetPose().Pos().Y() - 0.4, 0);
            auto max = ignition::math::Vector3d(new_vehicle->GetPose().Pos().X() + 0.4, new_vehicle->GetPose().Pos().Y() + 0.4, 0);
            auto box = ignition::math::Box(min,max);
            auto new_node = QTData(box, new_vehicle, vehicle_type);
            this->vehicle_quadtree->Insert(new_node);
            continue;
        } 
      
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
        
    }

    //this->static_quadtree->Print();
    //this->vehicle_quadtree->Print();
}

void Puppeteer::OnUpdate(const gazebo::common::UpdateInfo &_info){
    double dt = (_info.simTime - this->last_update).Double();

    if (dt < 1/this->update_freq){
        return;
    }

    this->last_update = _info.simTime;

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

        auto min = ignition::math::Vector3d(vehicle->GetPose().Pos().X() - 2, vehicle->GetPose().Pos().Y() - 2, 0);
        auto max = ignition::math::Vector3d(vehicle->GetPose().Pos().X() + 2, vehicle->GetPose().Pos().Y() + 2, 0);
        auto query_range = ignition::math::Box(min,max);

        std::vector<QTData> nearby_objects = this->static_quadtree->QueryRange(query_range);
        for (auto n: nearby_objects){
            near_objects.push_back(boost::static_pointer_cast<gazebo::physics::Entity>(n.data));
            
        }
        std::vector<QTData> nearby_vehicles = this->vehicle_quadtree->QueryRange(query_range);
        for (auto n: nearby_vehicles){
            near_vehicles.push_back(boost::static_pointer_cast<Vehicle>(n.data));
        }

        vehicle->OnUpdate(_info, dt, near_vehicles, near_objects);
    }
}

void Puppeteer::ReadSDF(){
    if (this->sdf->HasElement("building_name")){
        this->building_name =this->sdf->GetElement("building_name")->Get<std::string>();
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
                    standing_duration = std::stod(actor_info["walking_duration"]);
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

        } /*else if (actor_info["vehicle_type"] == "follower"){

            std::string leader_name = "";
            std::cout << actor_info["vehicle_type"] << std::endl;
            if (actor_info.find("leader") != actor_info.end()){
                leader_name = actor_info["leader"];
                std::cout << actor_info["leader"] << std::endl;;
                res = boost::make_shared<Follower>(actor, 1, 10, max_speed, actor->WorldPose(), ignition::math::Vector3d(0,0,0), this->collision_entities, this->vehicles, leader_name); // read in as params 
            } else{
                std::cout << "leader name not found\n";
            }
        }*/
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

//VEHICLE CLASS

Vehicle::Vehicle(gazebo::physics::ActorPtr _actor, 
double _mass, 
double _max_force, 
double _max_speed, 
ignition::math::Pose3d initial_pose, 
ignition::math::Vector3d initial_velocity,
std::vector<gazebo::physics::EntityPtr> objects){
    
    this->actor = _actor;
    this->mass = _mass;
    this->max_force = _max_force;
    this->max_speed = _max_speed;
    this->pose = initial_pose;
    this->curr_target = initial_pose.Pos();
    this->velocity = initial_velocity;
    this->acceleration = 0;
    this->all_objects = objects;

    std::map<std::string, gazebo::common::SkeletonAnimation *>::iterator it;
    std::map<std::string, gazebo::common::SkeletonAnimation *> skel_anims = this->actor->SkeletonAnimations();

    for ( it = skel_anims.begin(); it != skel_anims.end(); it++ ){   
        this->trajectories[it->first] = std::make_shared<gazebo::physics::TrajectoryInfo>();
        this->trajectories[it->first]->type = it->first;
        this->trajectories[it->first]->duration = 1.0;
    }

    this->actor->SetCustomTrajectory(this->trajectories["walking"]);
}

void Vehicle::OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){
    this->Seek(this->curr_target);
    this->UpdateModel();
    this->UpdatePosition(dt);
}

gazebo::physics::ActorPtr Vehicle::GetActor(){
    return this->actor;
}

void Vehicle::UpdateModel(){

    double distance_travelled = (this->pose.Pos() - this->actor->WorldPose().Pos()).Length();
	this->actor->SetWorldPose(this->pose, true, true);
	this->actor->SetScriptTime(this->actor->ScriptTime() + (distance_travelled * this->animation_factor));
}

void Vehicle::ApplyForce(ignition::math::Vector3d force){
    this->acceleration+=(force/this->mass);
}

void Vehicle::UpdatePosition(double dt){
    this->velocity+=this->acceleration*dt;
    
    if (this->velocity.Length() > this->max_speed){
        this->velocity.Normalize();
        this->velocity*=this->max_speed;
    }

    ignition::math::Vector3d direction = this->velocity;

    direction.Normalize();

    double dir_yaw = atan2(direction.Y(), direction.X());

    double current_yaw = this->pose.Rot().Yaw();

    ignition::math::Angle yaw_diff = dir_yaw-current_yaw+IGN_PI_2;
   
    yaw_diff.Normalize();

    if (this->velocity.Length()<10e-2){
        yaw_diff = 0;
    }
   
    this->pose.Pos() += this->velocity*dt;

    this->pose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, current_yaw + yaw_diff.Radian()*0.1);

    this->acceleration = 0;

}

void Vehicle::Seek(ignition::math::Vector3d target, double weight){

    ignition::math::Vector3d desired_v = target-this->pose.Pos();
    desired_v.Normalize();
    desired_v*=this->max_speed;

    ignition::math::Vector3d steer = desired_v - this->velocity;

    steer *= weight;
    if (steer.Length() > this->max_force){
        steer.Normalize();
        steer*=this->max_force;
    }

    ApplyForce(steer);
}

void Vehicle::Arrival(ignition::math::Vector3d target, double weight){ 

    ignition::math::Vector3d desired_v = target-this->pose.Pos();
    double dist = desired_v.Length();
    desired_v.Normalize();

    if (dist <this->slowing_distance){
        desired_v*=(dist/this->slowing_distance)*(this->max_speed);
    }else{
        desired_v*=this->max_speed;
    }


    ignition::math::Vector3d steer = desired_v - this->velocity;

    steer *= weight;

    if (steer.Length() > this->max_force){
        steer.Normalize();
        steer*=this->max_force;
    }

    ApplyForce(steer);
}

void Vehicle::AvoidObstacles(std::vector<gazebo::physics::EntityPtr> objects){

    ignition::math::Vector3d boundary_force = ignition::math::Vector3d(0,0,0);
    for (gazebo::physics::EntityPtr object: objects){
        ignition::math::Box box = object->BoundingBox();

        double min_z = std::min(box.Min().Z(), box.Max().Z());
        if (min_z > 1.5){
            continue;
        }
        ignition::math::Vector3d min_normal = utilities::min_repulsive_vector(this->pose.Pos(), object);
	
		double dist = min_normal.Length();
		if (dist < this->obstacle_margin){
			min_normal.Normalize();
			boundary_force += min_normal/(dist*dist);
		}
    }

    this->ApplyForce(boundary_force);
}

void Vehicle::AvoidActors(std::vector<boost::shared_ptr<Vehicle>> vehicles){

    ignition::math::Vector3d steer = ignition::math::Vector3d(0,0,0);
    for (int i =0; i<(int) vehicles.size(); i++){
        auto other = vehicles[i]->GetActor();
        if (other == this->actor){
            continue;
        }
        ignition::math::Vector3d this_pos = this->pose.Pos();
		this_pos.Z() = 0;
		ignition::math::Vector3d other_pos = other->WorldPose().Pos();
		other_pos.Z() = 0;
		ignition::math::Vector3d rad = this_pos-other_pos;
		double dist = rad.Length();
		
		if (dist<this->obstacle_margin){
			rad.Normalize();	
			rad/=dist;
			steer += rad;
		}
    }

    if (steer.Length()>this->max_force){
			steer.Normalize();
			steer*=this->max_force;
	}

    this->ApplyForce(steer);
}

void Vehicle::SetAllObjects(std::vector<gazebo::physics::EntityPtr> objects){
    this->all_objects = objects;
}

ignition::math::Pose3d Vehicle::GetPose(){
    return this->pose;
}

ignition::math::Vector3d Vehicle::GetVelocity(){
    return this->velocity;
}

std::string Vehicle::GetName(){
    return this->actor->GetName();
}

// WANDERER

void Wanderer::OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){

    this->SetNextTarget();
    this->Seek(this->curr_target);
    this->AvoidActors(vehicles);
    this->AvoidObstacles(objects);
    
    this->UpdatePosition(dt);
    this->UpdateModel();
}

void Wanderer::SetNextTarget(){
    this->curr_theta += ignition::math::Rand::DblUniform(-this->rand_amp,this->rand_amp); //TODO: perlin noise

    auto dir = this->velocity;
    
    dir.Normalize();
    
    dir*=2;
    
    auto offset =  ignition::math::Vector3d(1,0,0);
    
    auto rotation = ignition::math::Quaterniond(0,0,this->curr_theta);

    offset = rotation.RotateVector(offset);

    this->curr_target = this->pose.Pos() + dir + offset;

}

// RANDOM WALKER

void RandomWalker::OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){


    if ((this->pose.Pos() - this->curr_target).Length()<this->arrival_distance){
        this->SetNextTarget(this->all_objects);
    }

    
    this->Seek(this->curr_target);
    this->AvoidActors(vehicles);
    this->AvoidObstacles(objects);//TODO: make sure this is safe here
    
    this->UpdatePosition(dt);
    this->UpdateModel();
}

void RandomWalker::SetNextTarget(std::vector<gazebo::physics::EntityPtr> objects){
    bool target_found = false;

    while (!target_found){
        
        auto dir = this->velocity;
		if (dir.Length() < 1e-6){
			dir = ignition::math::Vector3d(ignition::math::Rand::DblUniform(-1, 1),ignition::math::Rand::DblUniform(-1, 1),0);
		}
        dir.Normalize();
        auto rotation =  ignition::math::Quaterniond::EulerToQuaternion(0,0,ignition::math::Rand::DblUniform(-3, 3)); 
		dir = rotation.RotateVector(dir);
        dir*=10000;
		auto ray = ignition::math::Line3d(this->pose.Pos().X(), this->pose.Pos().Y(), this->pose.Pos().X() + dir.X(), this->pose.Pos().Y() + dir.Y());
		ignition::math::Vector3d closest_intersection;
        ignition::math::Line3d closest_edge;
		double min_dist = 100000;

        for (auto object: objects){
            std::vector<ignition::math::Line3d> edges = utilities::get_edges(object);

            for (ignition::math::Line3d edge: edges){
				ignition::math::Vector3d test_intersection;

				if (ray.Intersect(edge, test_intersection)){ //if the ray intersects the boundary
                    ignition::math::Vector3d zero_z = this->pose.Pos();
                    zero_z.Z() = 0;
					double dist_to_int = (test_intersection-zero_z).Length();
					if (dist_to_int < min_dist){
						min_dist = dist_to_int;
						closest_intersection = test_intersection;
                        closest_edge = edge;
					}

				}
						
			}
        }

        auto zero_z = this->pose.Pos();
        zero_z.Z() = 0;
        auto final_ray = closest_intersection - zero_z;
		auto v_to_add = final_ray*ignition::math::Rand::DblUniform(0.1,0.9);

        ignition::math::Vector3d normal;
        if (utilities::get_normal_to_edge(this->pose.Pos(), closest_edge, normal) && (normal.Length() < this->obstacle_margin)){
            continue;
        } 

        if ((final_ray-v_to_add).Length() < (this->obstacle_margin)){ 
			v_to_add.Normalize();
			auto small_subtraction = (v_to_add*this->obstacle_margin)*2;
			v_to_add = final_ray - small_subtraction;
			if (small_subtraction.Length() > final_ray.Length()){
				v_to_add*=0;
			}
		}
		
		this->curr_target = v_to_add + this->pose.Pos();
		target_found = true;

    }

    
}

// BOID 

Boid::Boid(gazebo::physics::ActorPtr _actor, 
double _mass, 
double _max_force, 
double _max_speed, 
ignition::math::Pose3d initial_pose, 
ignition::math::Vector3d initial_velocity, 
std::vector<gazebo::physics::EntityPtr> objects, 
double _alignement, 
double _cohesion, 
double _separation,
double angle,
double radius)
: Vehicle(_actor, _mass, _max_force, _max_speed, initial_pose, initial_velocity, objects){
    this->weights[ALI] = _alignement;
    this->weights[COH] = _cohesion;
    this->weights[SEP] = _separation;

    this->FOV_angle = angle;
    this->FOV_radius = radius;
}

void Boid::OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){


    this->Alignement(dt, vehicles);
    this->Cohesion(vehicles);
    this->Separation(vehicles);
    this->AvoidObstacles(objects);

    this->UpdatePosition(dt);
    this->UpdateModel();
}

void Boid::Separation(std::vector<boost::shared_ptr<Vehicle>> vehicles){
    ignition::math::Vector3d steer = ignition::math::Vector3d(0,0,0);
    for (auto other: vehicles){
        if (other->GetName() == this->GetName()){
            continue;
        }
        auto this_pos = this->pose.Pos();
		this_pos.Z() = 0;
		auto other_pos = other->GetPose().Pos();
		other_pos.Z() = 0;
		auto rad = this_pos-other_pos;
		double dist = rad.Length();

        auto dir = this->velocity;
        dir.Normalize();
        rad.Normalize();
        double angle = std::acos(rad.Dot(dir));
		
		if (dist<this->obstacle_margin){
			rad.Normalize();	
			rad/=dist;
			steer += rad;
		}
    }
    
    if ((steer*this->weights[SEP]).Length() > this->max_force){
        steer.Normalize();
        steer*=this->max_force;
    } else{
        steer*= this->weights[SEP];
    }

    this->ApplyForce(steer);
}

void Boid::Alignement(double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles){
    auto vel_sum = ignition::math::Vector3d(0,0,0);
    int count = 0;

    for (auto other : vehicles){
        if (other->GetName() == this->GetName()){
            continue;
        }
        auto this_pos = this->pose.Pos();
		this_pos.Z() = 0;
		auto other_pos = other->GetPose().Pos();
		other_pos.Z() = 0;
		auto r = this_pos-other_pos;

        if (r.Length() < this->FOV_radius){

            auto dir = this->velocity;
            dir.Normalize();
            r.Normalize();
            double angle = std::acos(r.Dot(dir));

            if (angle<this->FOV_angle/2){
                vel_sum+= other->GetVelocity();
                count++;

            }
        }
        
    }

    if (count >0){
		
		// Implement Reynolds: Steering = Desired - Velocity
		vel_sum.Normalize();
		vel_sum*=this->max_speed;
		vel_sum-=this->velocity;

        if ((vel_sum*this->weights[SEP]).Length() > this->max_force){
            vel_sum.Normalize();
            vel_sum*=this->max_force;
        } else{
            vel_sum*= this->weights[SEP];
        }

	    this->ApplyForce(vel_sum);
	}
}

void Boid::Cohesion(std::vector<boost::shared_ptr<Vehicle>> vehicles){
    auto sum_pos = ignition::math::Vector3d(0,0,0);
    int count = 0;
    auto this_pos = this->pose.Pos();
	this_pos.Z() = 0;

    for (auto other: vehicles){
        if (other->GetName() == this->GetName()){
            continue;
        }
		auto other_pos = other->GetPose().Pos();
		other_pos.Z() = 0;
		auto rad = this_pos-other_pos;
		double dist = rad.Length();
        auto dir = this->velocity;
        dir.Normalize();
        rad.Normalize();
        double angle = std::acos(rad.Dot(dir));
		if (dist<this->FOV_radius && angle<this->FOV_angle/2){
            sum_pos+=other_pos;
            count++;
        }
    }

    if (count >0){
        this->Seek(sum_pos, this->weights[COH]);
    }
}

/// STANDER

Stander::Stander(gazebo::physics::ActorPtr _actor, 
double _mass, 
double _max_force, 
double _max_speed, 
ignition::math::Pose3d initial_pose, 
ignition::math::Vector3d initial_velocity,  
std::vector<gazebo::physics::EntityPtr> objects, 
double _standing_duration,
double _walking_duration)
: Wanderer(_actor, _mass, _max_force, _max_speed, initial_pose, initial_velocity, objects){

    this->standing_duration = std::max(0.0 ,_standing_duration +  ignition::math::Rand::DblUniform(-0.5,0.5));
    this->walking_duration = std::max(0.0 , _walking_duration +  ignition::math::Rand::DblUniform(-0.5,0.5));

    this->standing = (bool) ignition::math::Rand::IntUniform(0,1);

    if (walking_duration <= 0){
        this->never_walk = true;
    }
    if (standing){
        this->actor->SetCustomTrajectory(this->trajectories["standing"]);
    } else{
        this->actor->SetCustomTrajectory(this->trajectories["walking"]);
    }
    this->UpdatePosition(0.1);
    this->actor->SetWorldPose(this->pose, true, true);
    this->actor->SetScriptTime(this->actor->ScriptTime());
}

void Stander::UpdateModel(double dt){

    if (this->standing){
        this->actor->SetWorldPose(this->pose, true, true);
	    this->actor->SetScriptTime(this->actor->ScriptTime() + dt*this->animation_factor);
    } else{
        double distance_travelled = (this->pose.Pos() - this->actor->WorldPose().Pos()).Length();
	    this->actor->SetWorldPose(this->pose, true, true);
	    this->actor->SetScriptTime(this->actor->ScriptTime() + (distance_travelled * this->animation_factor));
    }
    
}

void Stander::OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){

    if (this->standing){

        if(!this->never_walk && (_info.simTime - this->standing_start).Double() >= this->standing_duration){
            this->standing = false;
            this->walking_start = _info.simTime;
            this->actor->SetCustomTrajectory(this->trajectories["walking"]);
            this->standing_duration += ignition::math::Rand::DblUniform(-0.5,0.5);
            if (this->standing_duration <= 0){
                this->standing_duration = 0;
            }
        }
    } else{

        this->SetNextTarget();
        this->Seek(this->curr_target);
        this->AvoidActors(vehicles);
        this->AvoidObstacles(objects);
    
        this->UpdatePosition(dt);

        if((_info.simTime - this->walking_start).Double() >= this->walking_duration){
            this->standing = true;
            this->standing_start = _info.simTime;
            this->actor->SetCustomTrajectory(this->trajectories["standing"]);
            this->walking_duration += ignition::math::Rand::DblUniform(-0.5,0.5);
            if (this->walking_duration <= 0){
                this->walking_duration = 0;
            }
        }
    }
 
    this->UpdateModel(dt);
}


Sitter::Sitter(gazebo::physics::ActorPtr _actor, std::string _chair_name, std::vector<gazebo::physics::EntityPtr> objects, double height)
: Vehicle(_actor, 1, 1, 1, ignition::math::Pose3d(100,100,0.5,0,0,0), ignition::math::Vector3d(0,0,0), objects){
    this->chair_name = _chair_name;
    bool found = false;
    for (auto model: this->all_objects){
        if (model->GetName() == this->chair_name){
            this->chair = model;
            found = true;
        }
    }

    if (found) {
    
        this->pose = this->chair->WorldPose();
        this->pose.Pos().Z()= height;
        this->pose.Rot() = ignition::math::Quaterniond(1.15, 0, this->chair->WorldPose().Rot().Yaw());
        this->actor->SetCustomTrajectory(this->trajectories["sitting"]);
        this->actor->SetWorldPose(this->pose, true, true);
        this->actor->SetScriptTime(this->actor->ScriptTime());
    }
}

void Sitter::OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){
    this->UpdateModel(dt);
}

void Sitter::UpdateModel(double dt){

    this->actor->SetWorldPose(this->pose, true, true);
	this->actor->SetScriptTime(this->actor->ScriptTime() + dt*this->animation_factor);
    
}


Follower::Follower(gazebo::physics::ActorPtr _actor,
double _mass,
double _max_force, 
double _max_speed, 
ignition::math::Pose3d initial_pose, 
ignition::math::Vector3d initial_velocity, 
std::vector<gazebo::physics::EntityPtr> objects,
std::vector<boost::shared_ptr<Vehicle>> vehicles, 
std::string _leader_name)
: Vehicle(_actor, _mass, _max_force, _max_speed, initial_pose, initial_velocity, objects){

    this->leader_name = _leader_name;
    for (auto other: vehicles){
        if (other->GetName() == this->leader_name){
            this->leader = other;
            std::cout << "YAY\n";
        }
    }

}


void Follower::SetNextTarget(double dt){
    auto leader_dir = this->leader->GetVelocity();

    if (leader_dir.Length() < 10e-6){
    
        auto rotation = ignition::math::Quaterniond(0,0,this->rand_angle);
        auto offset = rotation.RotateVector(ignition::math::Vector3d(1,0,0));
        this->curr_target = this->leader->GetPose().Pos() - offset;
       
        return;
    }

    leader_dir.Normalize();

    // if we find ourselves in front of the leader, steer laterally away from the leaders path 

    auto front_edge = ignition::math::Line3d(this->leader->GetPose().Pos(), this->leader->GetPose().Pos() + leader_dir);

    ignition::math::Vector3d normal;

    if (utilities::get_normal_to_edge(this->pose.Pos(), front_edge, normal)){
        if (normal.Length() < this->obstacle_margin){
            auto mag = normal.Length();
            normal.Normalize();
            normal *= 1/(mag*mag);
            if (normal.Length() > this->max_force){
                normal.Normalize();
                normal*=this->max_force;
            }
            this->ApplyForce(normal);
        }
    }

    this->curr_target = this->leader->GetPose().Pos() - leader_dir/2;

}


void Follower::OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){

    
    this->SetNextTarget(dt);
    this->Arrival(this->curr_target);
    
    this->AvoidActors(vehicles);
    this->AvoidObstacles(objects);
    
    this->UpdatePosition(dt);
    this->UpdateModel();
}

