#include "puppeteer.hh"
#include "utilities.hh"

GZ_REGISTER_WORLD_PLUGIN(Puppeteer);

//PUPPETEER CLASS

void Puppeteer::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){

    this->world = _world;
    this->sdf = _sdf;
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&Puppeteer::OnUpdate, this, std::placeholders::_1));

    this->ReadSDF();

    for (unsigned int i = 0; i < world->ModelCount(); ++i) {
        auto model = world->ModelByIndex(i);
        //std::cout << model->GetName() << std::endl;

        auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

        if (act){
            //std::string prefix = "";
            //std::cout << act->GetSDF()->ToString(prefix) << std::endl;
            this->vehicles.push_back(this->CreateVehicle(act));
            continue;
        } 

        if (model->GetName() == this->building_name){
            auto links = model->GetLinks();
            for (gazebo::physics::LinkPtr link: links){
                std::vector<gazebo::physics::CollisionPtr> collision_boxes = link->GetCollisions();
                for (gazebo::physics::CollisionPtr collision_box: collision_boxes){
                    this->collision_entities.push_back(collision_box); //TODO: check if this is correct (maybe do dynamic pointer cast )
                }
                    
            }
        } else if (model->GetName() != "ground_plane"){
            this->collision_entities.push_back(model);
        }
    }


}


void Puppeteer::OnUpdate(const gazebo::common::UpdateInfo &_info){
    double dt = (_info.simTime - this->last_update).Double();

    if (dt < 1/this->update_freq){
        return;
    }

    this->last_update = _info.simTime;

    for (auto vehicle: this->vehicles){
        vehicle->OnUpdate(_info, dt, this->vehicles, this->collision_entities);
    }
}

void Puppeteer::ReadSDF(){
    if (this->sdf->HasElement("building_name")){
        this->building_name =this->sdf->GetElement("building_name")->Get<std::string>();
    }
}

std::shared_ptr<Vehicle> Puppeteer::CreateVehicle(gazebo::physics::ActorPtr actor){

    std::shared_ptr<Vehicle> res;
   
    auto sdf = actor->GetSDF();

    std::map<std::string, std::string> actor_info; 

    auto attribute = sdf->GetElement("plugin");
        
	while (attribute){
		actor_info[attribute->GetAttribute("name")->GetAsString()] = attribute->GetAttribute("filename")->GetAsString();
		attribute = attribute->GetNextElement("plugin");
	}

    if (actor_info.find("vehicle_type")!=actor_info.end()){
        std::cout << actor_info["vehicle_type"] << std::endl;
        if (actor_info["vehicle_type"] == "wanderer"){
            res = std::make_shared<Wanderer>(actor, 1, 10, 1, actor->WorldPose(), ignition::math::Vector3d(0,0,0));
        }
    }

    return res;
}

//VEHICLE CLASS

Vehicle::Vehicle(gazebo::physics::ActorPtr _actor, 
double _mass, 
double _max_force, 
double _max_speed, 
ignition::math::Pose3d initial_pose, 
ignition::math::Vector3d initial_velocity){
    
    this->actor = _actor;
    this->mass = _mass;
    this->max_force = _max_force;
    this->max_speed = _max_speed;
    this->pose = initial_pose;
    this->velocity = initial_velocity;
    this->acceleration = 0;

    std::map<std::string, gazebo::common::SkeletonAnimation *>::iterator it;
    std::map<std::string, gazebo::common::SkeletonAnimation *> skel_anims = this->actor->SkeletonAnimations();

    for ( it = skel_anims.begin(); it != skel_anims.end(); it++ ){   
        this->trajectories[it->first] = std::make_shared<gazebo::physics::TrajectoryInfo>();
        this->trajectories[it->first]->type = it->first;
        this->trajectories[it->first]->duration = 1.0;
    }

    this->actor->SetCustomTrajectory(this->trajectories["walking"]);
}

void Vehicle::OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<std::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){
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

void Vehicle::Seek(ignition::math::Vector3d target){

    ignition::math::Vector3d desired_v = target-this->pose.Pos();
    desired_v.Normalize();
    desired_v*=this->max_speed;

    ignition::math::Vector3d steer = desired_v - this->velocity;

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

void Vehicle::AvoidActors(std::vector<std::shared_ptr<Vehicle>> vehicles){

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

// WANDERER

void Wanderer::OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<std::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){

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
