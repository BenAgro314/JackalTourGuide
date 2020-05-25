#include "vehicles.hh"

// VEHICLE CLASS

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

            if (act && act == this->actor){
                continue;
            } else if (act){
                this->actors.push_back(act);
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

    if (boundary_force.Length() >0){
		boundary_force.Normalize();
		boundary_force*=this->max_speed;
		boundary_force-=this->velocity;
		if (boundary_force.Length()>this->max_force){
			boundary_force.Normalize();
			boundary_force*=this->max_force;
		}
	}

    this->ApplyForce(boundary_force);
}

void Vehicle::AvoidActors(){

    ignition::math::Vector3d steer = ignition::math::Vector3d(0,0,0);

    for (gazebo::physics::ActorPtr other: this->actors){
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

    if (steer.Length() >0){
		steer.Normalize();
		steer*=this->max_speed;
		steer-=this->velocity;
		if (steer.Length()>this->max_force){
			steer.Normalize();
			steer*=this->max_force;
		}
	}

    this->ApplyForce(steer);
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
    this->Seek(this->curr_target);
    this->AvoidActors();
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

// BOID CLASS

void Boid::OnUpdate(const gazebo::common::UpdateInfo &_inf){
    double dt = (_inf.simTime - this->last_update).Double();

    if (dt < 1/this->update_freq){
        return;
    }

    this->last_update = _inf.simTime;

    this->Alignement(dt);
    this->Cohesion();
    this->Separation();
    this->AvoidObstacles();

    this->UpdatePosition(dt);
    this->UpdateModel();
}


Boid::Boid(gazebo::physics::ActorPtr _actor, 
double _mass, 
double _max_force, 
double _max_speed, 
ignition::math::Pose3d initial_pose, 
ignition::math::Vector3d initial_velocity, 
std::string animation, 
std::string _building_name, 
double _alignement, 
double _cohesion, 
double _separation,
double angle,
double radius)
: Vehicle(_actor, _mass, _max_force, _max_speed, initial_pose, initial_velocity, animation, _building_name){
    this->weights[ALI] = _alignement;
    this->weights[COH] = _cohesion;
    this->weights[SEP] = _separation;

    for (gazebo::physics::ActorPtr other: this->actors){
        this->last_pos[other] = other->WorldPose().Pos();
    }

    this->FOV_angle = angle;
    this->FOV_radius = radius;
}

void Boid::Separation(){
    ignition::math::Vector3d steer = ignition::math::Vector3d(0,0,0);

    for (gazebo::physics::ActorPtr other: this->actors){
        ignition::math::Vector3d this_pos = this->pose.Pos();
		this_pos.Z() = 0;
		ignition::math::Vector3d other_pos = other->WorldPose().Pos();
		other_pos.Z() = 0;
		ignition::math::Vector3d rad = this_pos-other_pos;
		double dist = rad.Length();

        ignition::math::Vector3d dir = this->velocity;
        dir.Normalize();
        rad.Normalize();
        double angle = std::acos(rad.Dot(dir));
		
		if (dist<this->obstacle_margin){// && angle<this->FOV_angle/2){
			rad.Normalize();	
			rad/=dist;
			steer += rad;
		}
    }

    if (steer.Length() >0){
		steer.Normalize();
		steer*=this->max_speed;
		steer-=this->velocity;
		if (steer.Length()>this->max_force){
			steer.Normalize();
			steer*=this->max_force;
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

void Boid::Alignement(double dt){
    ignition::math::Vector3d vel_sum = ignition::math::Vector3d(0,0,0);
    int count = 0;

    for (gazebo::physics::ActorPtr other : this->actors){

        

        ignition::math::Vector3d this_pos = this->pose.Pos();
		this_pos.Z() = 0;
		ignition::math::Vector3d other_pos = other->WorldPose().Pos();
		other_pos.Z() = 0;
		ignition::math::Vector3d r = this_pos-other_pos;

        if (r.Length() < this->FOV_radius*2){
            ignition::math::Vector3d past_pos = this->last_pos[other]; //track the pos of all actors in 2* the FOV radius 
		    this->last_pos[other] = other->WorldPose().Pos();

            if (r.Length() < this->FOV_radius){

                ignition::math::Vector3d dir = this->velocity;
                dir.Normalize();
                r.Normalize();
                double angle = std::acos(r.Dot(dir));

                if (angle<this->FOV_angle/2){
                    
                    ignition::math::Vector3d vel = (other->WorldPose().Pos()-past_pos)/dt;
                    count++;
                    
                    vel_sum+= vel;
                }
            }
        }

        
    }

    if (count >0){
		
		// Implement Reynolds: Steering = Desired - Velocity
		vel_sum.Normalize();
		vel_sum*=this->max_speed;
		vel_sum-=this->velocity;
		if (vel_sum.Length() > this->max_force){
			vel_sum.Normalize();
			vel_sum*=this->max_force;
		}

        if ((vel_sum*this->weights[SEP]).Length() > this->max_force){
            vel_sum.Normalize();
            vel_sum*=this->max_force;
        } else{
            vel_sum*= this->weights[SEP];
        }

	    this->ApplyForce(vel_sum);
	}

   
}

void Boid::Cohesion(){
    ignition::math::Vector3d sum_pos = ignition::math::Vector3d(0,0,0);

    int count = 0;

    ignition::math::Vector3d this_pos = this->pose.Pos();
	this_pos.Z() = 0;

    for (gazebo::physics::ActorPtr other: this->actors){
        
		ignition::math::Vector3d other_pos = other->WorldPose().Pos();
		other_pos.Z() = 0;
		ignition::math::Vector3d rad = this_pos-other_pos;
		double dist = rad.Length();

        ignition::math::Vector3d dir = this->velocity;
        dir.Normalize();
        rad.Normalize();
        double angle = std::acos(rad.Dot(dir));
		
		if (dist<this->FOV_radius && angle<this->FOV_angle/2){
            sum_pos+=other_pos;
            count++;
        }
    }

    if (count >0){
        ignition::math::Vector3d desired_v = sum_pos-this_pos;
        desired_v.Normalize();
        desired_v*=this->max_speed;

        ignition::math::Vector3d steer = desired_v - this->velocity;

        if ((steer*this->weights[COH]).Length() > this->max_force){
            steer.Normalize();
            steer*=this->max_force;
        } else{
            steer*=this->weights[COH];
        }

        ApplyForce(steer);
    }
}

//RANDOM WALKER CLASS (depreciated, should use wanderer)

void RandomWalker::OnUpdate(const gazebo::common::UpdateInfo &_inf){

    double dt = (_inf.simTime - this->last_update).Double();

    if (dt < 1/this->update_freq){
        return;
    }

    this->last_update = _inf.simTime;

    if ((this->pose.Pos() - this->curr_target).Length()<this->arrival_distance){
        this->SetNextTarget();
    }

    
    this->Seek(this->curr_target);
    this->AvoidActors();
    
    this->UpdatePosition(dt);
    this->UpdateModel();
}

void RandomWalker::SetNextTarget(){
    bool target_found = false;

    while (!target_found){
        
        ignition::math::Vector3d dir = this->velocity;
		if (dir.Length() < 1e-6){
			dir = ignition::math::Vector3d(ignition::math::Rand::DblUniform(-1, 1),ignition::math::Rand::DblUniform(-1, 1),0);
		}
        dir.Normalize();
        ignition::math::Quaterniond rotation =  ignition::math::Quaterniond::EulerToQuaternion(0,0,ignition::math::Rand::DblUniform(-3, 3)); 
		dir = rotation.RotateVector(dir);
        dir*=10000;
		ignition::math::Line3d ray = ignition::math::Line3d(this->pose.Pos().X(), this->pose.Pos().Y(), this->pose.Pos().X() + dir.X(), this->pose.Pos().Y() + dir.Y());
		ignition::math::Vector3d closest_intersection;
        ignition::math::Line3d closest_edge;
		double min_dist = 100000;

        for (gazebo::physics::EntityPtr object: this->objects){
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

        ignition::math::Vector3d zero_z = this->pose.Pos();
        zero_z.Z() = 0;
        ignition::math::Vector3d final_ray = closest_intersection - zero_z;
		ignition::math::Vector3d v_to_add = final_ray*ignition::math::Rand::DblUniform(0.1,0.9);


        //TODO: project against closest edge to check if we are too close 
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

PathFollower::PathFollower(gazebo::physics::ActorPtr _actor,
    double _mass,
    double _max_force, 
    double _max_speed, 
    ignition::math::Pose3d initial_pose, 
    ignition::math::Vector3d initial_velocity, 
    std::string animation, 
    std::string _building_name, 
    utilities::Path _path)
: Vehicle(_actor, _mass, _max_force, _max_speed, initial_pose, initial_velocity, animation, _building_name){
    
    this->path = _path;
    
}

void PathFollower::OnUpdate(const gazebo::common::UpdateInfo &_inf){
    double dt = (_inf.simTime - this->last_update).Double();

    if (dt < 1/this->update_freq){
        return;
    }

    this->last_update = _inf.simTime;

    

    
    this->FollowPath(dt);
    this->AvoidActors();
    this->AvoidObstacles();
    
    this->UpdatePosition(dt);
    this->UpdateModel();
}

void PathFollower::FollowPath(double dt){
    ignition::math::Vector3d target;
    double min_normal_dist = 100000;

    for (int i =0; i<(int) this->path.points.size(); i++){
        ignition::math::Vector3d start = path.points[i];
        ignition::math::Vector3d end;
        if (i == (int) this->path.points.size()-1){
            end = path.points[0];
        } else{
            end = path.points[i+1];
        }

        ignition::math::Vector3d dir = this->velocity*dt;

        ignition::math::Vector3d predicted_loc = this->pose.Pos() + dir;

        ignition::math::Vector3d normal_vec;
        ignition::math::Vector3d normal_point;

        if(utilities::get_normal_to_edge(predicted_loc, ignition::math::Line3d(start,end), normal_vec)){
            normal_point = predicted_loc-normal_vec;
        } else{
            normal_point = end;
        }

        double dist = (normal_point-predicted_loc).Length();

        if (dist<min_normal_dist){
            min_normal_dist = dist;
            target = normal_point;
        }
    }

    this->Seek(target);
}
