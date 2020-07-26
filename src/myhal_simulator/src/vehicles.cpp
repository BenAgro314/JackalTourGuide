#include "vehicles.hh"
#include "Perlin.h"
#include <thread>

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
    this->height = initial_pose.Pos().Z();

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
    boundary_force.Z() =0;
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
		
		if (dist<this->obstacle_margin && dist > 0){
			rad.Normalize();	
			rad/=dist;
			steer += rad;
		}
    }

    if (steer.Length()>this->max_force){
			steer.Normalize();
			steer*=this->max_force;
	}
    steer.Z() = 0;
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

bool Vehicle::IsStill(){
    return this->still;
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
    this->curr_target.Z() = this->height;

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
           
            auto box = object->BoundingBox();

            std::vector<ignition::math::Line3d> edges = utilities::get_box_edges(box);

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
        this->curr_target.Z() = this->height;
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
		
		if (dist<this->obstacle_margin && dist > 0){
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
    steer.Z() = 0;
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
        vel_sum.Z() = 0;
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
        sum_pos.Z() = this->height;
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
double _walking_duration,
int start_mode)
: Wanderer(_actor, _mass, _max_force, _max_speed, initial_pose, initial_velocity, objects){

    this->standing_duration = std::max(0.0 ,_standing_duration +  ignition::math::Rand::DblUniform(-0.5,0.5));
    this->walking_duration = std::max(0.0 , _walking_duration +  ignition::math::Rand::DblUniform(-0.5,0.5));

    if (start_mode == 2){
        this->standing = (bool) ignition::math::Rand::IntUniform(0,1);
    } else if (start_mode == 1){
        this->standing = false;
    } else {
        this->standing = true;
    }
    

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
        this->still = true;
        if(!this->never_walk && (_info.simTime - this->standing_start).Double() >= this->standing_duration){
            this->standing = false;
            this->walking_start = _info.simTime;
            this->actor->SetCustomTrajectory(this->trajectories["walking"]);
            this->velocity = 0;
            this->standing_duration += ignition::math::Rand::DblUniform(-0.5,0.5);
            if (this->standing_duration <= 0){
                this->standing_duration = 0;
            }
        }
    } else{
        this->still = false;
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
    this->still = true;
    for (auto model: this->all_objects){
        if (model->GetParent()->GetParent()->GetName() == this->chair_name){
            this->chair = boost::static_pointer_cast<gazebo::physics::Entity>(model->GetParent()->GetParent());
            found = true;
            break;
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
std::string _leader_name, bool blocking)
: Vehicle(_actor, _mass, _max_force, _max_speed, initial_pose, initial_velocity, objects){

    this->leader_name = _leader_name;
    this->blocking = blocking;
    if (this->blocking){
        this->rand_angle_offset = ignition::math::Rand::DblUniform(-1,1);
    } else{
        this->rand_angle_offset = ignition::math::Rand::DblUniform(-2.5,2.5);
    }

}


void Follower::LoadLeader(gazebo::physics::EntityPtr leader){
    this->leader = leader;
    this->last_leader_pose = this->leader->WorldPose();
}


void Follower::SetNextTarget(double dt){

    if (this->leader == nullptr){
        this->curr_target = this->pose.Pos();
        return;
    }
    //auto leader_dir = this->leader->GetVelocity();
    auto leader_pose = this->leader->WorldPose();
    auto leader_dir = leader_pose.Pos() - this->last_leader_pose.Pos();
    
    auto rotation = ignition::math::Quaterniond(0,0,leader_pose.Rot().Yaw()+this->rand_angle_offset);

    if (leader_dir.Length() < 10e-6){
        
        auto offset = rotation.RotateVector(ignition::math::Vector3d(this->rand_radius,0,0));
        this->curr_target = leader_pose.Pos();
    
        if (this->blocking){
            this->curr_target += offset;
        } else{
            this->curr_target -= offset;
        }

        this->curr_target.Z() = this->pose.Pos().Z();
        
       
        return;
    }

    leader_dir.Normalize();

    // if we find ourselves in front of the leader, steer laterally away from the leaders path 

    if (!this->blocking){
        auto front_edge = ignition::math::Line3d(leader_pose.Pos(), leader_pose.Pos() + leader_dir);

        ignition::math::Vector3d normal;

        if (utilities::get_normal_to_edge(this->pose.Pos(), front_edge, normal)){
            if (normal.Length() < this->obstacle_margin){
                auto mag = normal.Length();
                if (mag == 0){
                    mag = 10e-9;
                }
                normal.Normalize();

                normal *= 1/(mag*mag);
                if (normal.Length() > this->max_force){
                    normal.Normalize();
                    normal*=this->max_force;
                }
                this->ApplyForce(normal);
            }
        }
    }

    leader_dir*=this->rand_radius;
    rotation = ignition::math::Quaterniond(0,0,this->rand_angle_offset);
    leader_dir = rotation.RotateVector(leader_dir);
    this->curr_target = leader_pose.Pos();

    if (this->blocking){
        this->curr_target+=leader_dir/2;
    }else{
        this->curr_target-=leader_dir/2;
    }

    this->curr_target.Z() = this->pose.Pos().Z();
    this->last_leader_pose = leader_pose;
}

void Follower::OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){

    
    this->SetNextTarget(dt);
    this->Arrival(this->curr_target);
    
    this->AvoidActors(vehicles);
    this->AvoidObstacles(objects);
    
    this->UpdatePosition(dt);
    this->UpdateModel();
}


void PathFollower::Follow(){

    if ((this->pose.Pos() - this->curr_target).Length() < this->arrival_distance){
        
        this->path_ind++;
        
        if (this->path_ind >= this->curr_path.size()){
            
            this->RePath();
        } 
            
        this->curr_target = this->curr_path[this->path_ind];
        this->curr_target.Z() = this->height;
        //std::cout << "TARGET: " << this->curr_target << std::endl;
    }
    
    this->Seek(this->curr_target);

}

PathFollower::PathFollower(gazebo::physics::ActorPtr _actor,
double _mass,
double _max_force, 
double _max_speed, 
ignition::math::Pose3d initial_pose, 
ignition::math::Vector3d initial_velocity, 
std::vector<gazebo::physics::EntityPtr> objects,
boost::shared_ptr<Costmap> costmap)
: Wanderer(_actor, _mass, _max_force, _max_speed, initial_pose, initial_velocity, objects){
    this->costmap = costmap;
    this->path_ind =0;
    
    this->RePath();
    //this->costmap->AStar(this->pose.Pos(), ignition::math::Vector3d(0,-10,this->height), this->curr_path);
    //this->curr_target = this->curr_path[this->path_ind];
    //std::cout << "size: " << this->curr_path.size() << std::endl;
}

void PathFollower::OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects){
    
    
    this->Follow();
    this->AvoidActors(vehicles);
    this->AvoidObstacles(objects);
    
    this->UpdatePosition(dt);
    this->UpdateModel();
}

void PathFollower::RePath(){
    this->path_ind = 1;

    ignition::math::Vector3d next_goal;
    this->curr_path.clear();
    do{
        next_goal = this->costmap->RandPos();
        //std::cout << "GOAL: " << next_goal << std::endl;
    } while (!this->costmap->AStar(this->pose.Pos(), next_goal, this->curr_path, false));
    
}
