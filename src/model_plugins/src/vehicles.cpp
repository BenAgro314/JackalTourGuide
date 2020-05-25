#include "vehicles.hh"
#include "utilities.hh"


Vehicle::Vehicle(gazebo::physics::ActorPtr _actor, 
    double _mass, 
    double _max_force,
    double _max_speed,
    ignition::math::Pose3d initial_pose, 
    ignition::math::Vector3d initial_velocity,
    std::string animation,
    std::string _building_name){

        this->actor = _actor;
        this->mass = _mass;
        this->max_force = _max_force;
        this->max_speed = _max_speed;
        this->pose = initial_pose;
        this->velocity = initial_velocity;
        this->acceleration = ignition::math::Vector3d(0,0,0);
        this->curr_target = initial_pose.Pos();
        

        auto skelAnims = this->actor->SkeletonAnimations();
  	    if (skelAnims.find(animation) == skelAnims.end()){
    	    std::cout << "Skeleton animation [" << animation << "] not found in Actor."
          	    << std::endl;
  	    }else{
            gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new gazebo::physics::TrajectoryInfo());
            trajectoryInfo->type = animation;
            trajectoryInfo->duration = 1.0;
            this->actor->SetCustomTrajectory(trajectoryInfo);
  	    }   

        this->building_name = _building_name;

        gazebo::physics::WorldPtr world = this->actor->GetWorld();

        this->initial_model_count = world->ModelCount();

        for (unsigned int i = 0; i < world->ModelCount(); ++i) {
            gazebo::physics::ModelPtr model = world->ModelByIndex(i);
            gazebo::physics::ActorPtr act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

            if (act){
                continue;
            }

            if (model->GetName()== this->building_name){
                std::vector<gazebo::physics::LinkPtr> links = model->GetLinks();
                for (gazebo::physics::LinkPtr link: links){
                    this->objects.push_back(link); //TODO: check if this is correct (maybe do dynamic pointer cast )
                }
            } else if (model->GetName() != "ground_plane"){
                this->objects.push_back(model);
            }
        }

    }

void Vehicle::AvoidObstacles(){
    gazebo::physics::WorldPtr world = this->actor->GetWorld();
    if (world->ModelCount() > this->initial_model_count){
        this->objects.push_back(world->ModelByIndex(this->initial_model_count++));
    }
    
    ignition::math::Vector3d boundary_force = ignition::math::Vector3d(0,0,0);
    for (gazebo::physics::EntityPtr object: this->objects){

        ignition::math::Vector3d min_normal = utilities::min_repulsive_vector(this->pose.Pos(), object);
	
		double dist = min_normal.Length();
		if (dist < this->obstacle_margin){
			min_normal.Normalize();
			boundary_force += min_normal/(dist*dist);
		}
    }

    this->ApplyForce(boundary_force);
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

void Vehicle::Arrival(ignition::math::Vector3d target){ //TODO: fix arrival to stop properly

    ignition::math::Vector3d desired_v = target-this->pose.Pos();
    double dist = desired_v.Length();
    desired_v.Normalize();

    if (dist <this->slowing_distance){
        desired_v*=(dist/this->slowing_distance)*(this->max_speed);
    }else{
        desired_v*=this->max_speed;
    }


    ignition::math::Vector3d steer = desired_v - this->velocity;

    if (steer.Length() > this->max_force){
        steer.Normalize();
        steer*=this->max_force;
    }

    ApplyForce(steer);
}

void Vehicle::OnUpdate(const gazebo::common::UpdateInfo &_inf){

    double dt = (_inf.simTime - this->last_update).Double();

    if (dt < 1/this->update_freq){
        return;
    }

    this->last_update = _inf.simTime;

    this->Arrival(this->curr_target);
    
    this->UpdatePosition(dt);
    this->UpdateModel();
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
    //TODO: fix oscillation
    this->pose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, current_yaw + yaw_diff.Radian()*0.1);
    

}

void Vehicle::UpdateModel(){
 
    double distance_travelled = (this->pose.Pos() - this->actor->WorldPose().Pos()).Length();
	this->actor->SetWorldPose(this->pose, true, true);
	this->actor->SetScriptTime(this->actor->ScriptTime() + (distance_travelled * this->animation_factor));
}


//WANDERER CLASS

void Wanderer::OnUpdate(const gazebo::common::UpdateInfo &_inf){

    

    double dt = (_inf.simTime - this->last_update).Double();

    if (dt < 1/this->update_freq){
        return;
    }

    this->last_update = _inf.simTime;

    
    this->SetNextTarget();
    this->Arrival(this->curr_target);
    this->AvoidObstacles();
    
    this->UpdatePosition(dt);
    this->UpdateModel();
}

void Wanderer::SetNextTarget(){
    this->curr_theta += ignition::math::Rand::DblUniform(-this->rand_amp,this->rand_amp); //TODO: perlin noise
    ignition::math::Vector3d dir = this->velocity;
    dir.Normalize();
    dir*=2;

    ignition::math::Vector3d offset =  ignition::math::Vector3d(1,0,0);
    ignition::math::Quaterniond rotation = ignition::math::Quaterniond(0,0,this->curr_theta);

    offset = rotation.RotateVector(offset);

    

    this->curr_target = this->pose.Pos() + dir + offset;

}
