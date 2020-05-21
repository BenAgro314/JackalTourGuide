
///TODO: RETARGET AFTER A CERTAIN AMOUNT OF TIME 

#include <functional>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Line2.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Box.hh>
#include <vector>
#include <string>
#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <map>

#include "randomwalk_plugin.hh"

using namespace gazebo;
using namespace servicesim;

GZ_REGISTER_MODEL_PLUGIN(servicesim::ActorPlugin)

class servicesim::ActorPluginPrivate
{

  /// \brief Pointer to the actor.
  public: physics::ActorPtr actor{nullptr};
  
  public: double max_force = 10;
  
  public: double mass = 0.5;
  
  public: ignition::math::Vector3d F_net = ignition::math::Vector3d(0,0,0);

  /// \brief Velocity of the actor
  public: double max_speed{2};
  
  public: ignition::math::Vector3d velocity = ignition::math::Vector3d(0,0,0);

  /// \brief List of connections such as WorldUpdateBegin
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief Radius in meters around target pose where we consider it was
  /// reached.
  public: double targetRadius{0.3};

  /// \brief Margin by which to increase an obstacle's bounding box on every
  /// direction (2x per axis).
  public: double obstacleMargin{0.2};

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  public: double animationFactor{5.1};

  /// \brief Time of the last update.
  public: common::Time lastUpdate;

  /// \brief List of models to avoid
  public: std::vector<std::string> obstacles;

  /// \brief Frequency in Hz to update
  public: double updateFreq{60};
  
  public: std::vector<ignition::math::Vector2d> polygon;
  
  public: ignition::math::Vector3d prev_target;
  
  public: ignition::math::Vector3d curr_target;
  
  public: common::Time last_target_time;
  
  public: bool follower = false;
  
  public: double dt;

  public: gazebo::physics::Link_V building_links;
};

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
    : dataPtr(new ActorPluginPrivate)
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Update frequency
  if (_sdf->HasElement("update_frequency"))
    this->dataPtr->updateFreq = _sdf->Get<double>("update_frequency");


  // Read in the velocity
  if (_sdf->HasElement("max_speed"))
    this->dataPtr->max_speed= _sdf->Get<double>("max_speed");


 
	auto pointElem = _sdf->GetElement("point");
	while (pointElem){
		
		this->dataPtr->polygon.push_back(pointElem->Get<ignition::math::Vector2d>());
		pointElem = pointElem->GetNextElement("point");
	}
	   
	   ignition::math::Pose3d holder;
	   if (_sdf->HasElement("pose")){
		    holder = _sdf->GetElement("pose")->Get<ignition::math::Pose3d>();
			this->dataPtr->actor->SetWorldPose(holder, false, false);
			this->dataPtr->curr_target = holder.Pos();
			this->dataPtr->curr_target.Z() = 0;
			this->dataPtr->prev_target = this->dataPtr->curr_target;
	   } 
		
  if (_sdf->HasElement("follower")){
	  this->dataPtr->follower = _sdf->Get<bool>("follower");
  } else{
	  this->dataPtr->follower = false;
  }
  /*
	if (_sdf->HasElement("velocity")){
		this->dataPtr->velocity = _sdf->GetElement("velocity")->Get<ignition::math::Vector3d>();
	} 
  */

  // Read in the target mradius
  if (_sdf->HasElement("target_radius"))
    this->dataPtr->targetRadius = _sdf->Get<double>("target_radius");

  // Read in the obstacle margin
  if (_sdf->HasElement("obstacle_margin"))
    this->dataPtr->obstacleMargin = _sdf->Get<double>("obstacle_margin");

  // Read in the animation factor
  if (_sdf->HasElement("animation_factor"))
    this->dataPtr->animationFactor = _sdf->Get<double>("animation_factor");

  // Read in the obstacles
  if (_sdf->HasElement("obstacle"))
  {
    auto obstacleElem = _sdf->GetElement("obstacle");
    while (obstacleElem)
    {
      auto name = obstacleElem->Get<std::string>();
      this->dataPtr->obstacles.push_back(name);
      obstacleElem = obstacleElem->GetNextElement("obstacle");
    }
  }

	///TODO: add running at a certain speed
  // Read in the animation name
  std::string animation{"animation"};
  if (_sdf->HasElement("animation"))
    animation = _sdf->Get<std::string>("animation");

  auto skelAnims = this->dataPtr->actor->SkeletonAnimations();
  if (skelAnims.find(animation) == skelAnims.end())
  {
    gzerr << "Skeleton animation [" << animation << "] not found in Actor."
          << std::endl;
  }
  else
  {
    // Set custom trajectory
    gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
    trajectoryInfo->type = animation;
    trajectoryInfo->duration = 1.0;

    this->dataPtr->actor->SetCustomTrajectory(trajectoryInfo);
  }

	auto world = this->dataPtr->actor->GetWorld();
   	for (unsigned int i = 0; i < world->ModelCount(); ++i) {
		// iterate over all models. Skip if the model is itself or if it needs to be ignored 
		
		auto model = world->ModelByIndex(i);
		auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

		if (model->GetName() == "myhal" || model->GetName() == "test_cell2"){ //add any desired building name
			//std::cout << "hi" << std::endl;
			this->dataPtr->building_links = model->GetLinks();
		}
   }
 
  
}




ignition::math::Vector3d ActorPlugin::WithinBounds(){
	//iterate over all boundaries:
	//if we are within a certain distance to the boundary, apply a normal force to the person 
	
	ignition::math::Pose3d actorPose = this->dataPtr->actor->WorldPose();
	ignition::math::Vector3d boundary_force = ignition::math::Vector3d(0,0,0);
	
	for (int i =0; i<(int) this->dataPtr->polygon.size(); i+=2){
		ignition::math::Vector3d boundary = ignition::math::Vector3d(this->dataPtr->polygon[i+1].X() - this->dataPtr->polygon[i].X(), this->dataPtr->polygon[i+1].Y() - this->dataPtr->polygon[i].Y(), 0);
		ignition::math::Vector3d pos = ignition::math::Vector3d(actorPose.Pos().X()-this->dataPtr->polygon[i].X(), actorPose.Pos().Y()-this->dataPtr->polygon[i].Y(), 0);
		// project pos vector onto boundary 
		ignition::math::Vector3d proj = ((pos.Dot(boundary))/(boundary.Dot(boundary)))*boundary;
		//get normal vector:
		ignition::math::Vector3d normal = pos-proj;
		double dist = normal.Length();
		if (dist < this->dataPtr->obstacleMargin){
			normal.Normalize();
			boundary_force += normal/(dist*dist);
		}
		
	}
	
	return boundary_force;
	
}


ignition::math::Vector3d ActorPlugin::ActorAvoidance(){
	ignition::math::Pose3d actorPose = this->dataPtr->actor->WorldPose();
	ignition::math::Vector3d steer = ignition::math::Vector3d(0,0,0);
	auto world = this->dataPtr->actor->GetWorld();
	int count = 0;
	
	for (unsigned int i = 0; i < world->ModelCount(); ++i) {
		// iterate over all models. Skip if the model is itself or if it needs to be ignored 
		
		auto model = world->ModelByIndex(i);
		auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
		
		if (!act) {
			continue;
		} if (act && act == this->dataPtr->actor){
			continue;
		}
		
		ignition::math::Vector3d modelPos = model->WorldPose().Pos();
		modelPos.Z() = 0;
		auto actorPos = actorPose.Pos();
		actorPos.Z() = 0;
		ignition::math::Vector3d rad = actorPos-modelPos;
		double dist = rad.Length();
		
		if (dist<this->dataPtr->obstacleMargin){
			rad.Normalize();	
			rad/=dist;
			steer += rad;
			count++;
		}
	}
	if (steer.Length() >0){
		steer.Normalize();
		steer*=this->dataPtr->max_speed;
		steer-=this->dataPtr->velocity;
		if (steer.Length()>this->dataPtr->max_force){
			steer.Normalize();
			steer*=this->dataPtr->max_force;
		}
	}
	
	return steer;

}

ignition::math::Vector3d ActorPlugin::ObstacleAvoidance(){
	ignition::math::Pose3d actorPose = this->dataPtr->actor->WorldPose();
	ignition::math::Vector3d boundary_force = ignition::math::Vector3d(0,0,0);
	auto world = this->dataPtr->actor->GetWorld();
	int count = 0;
	
	for (unsigned int i = 0; i < world->ModelCount(); ++i) {
		// iterate over all models. Skip if the model is itself or if it needs to be ignored 
		
		auto model = world->ModelByIndex(i);
		auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

		if (act || (model->GetName() == "myhal") || (model->GetName() == "ground_plane")){ 
			continue;
		}

		

		/*
		1. obtain bouding box of model
		2. for each edge of bouning box, find "the one we are closet too?" (the one with the closest perp distance and with a normal that intersects the edge)
		3. find normal pointing away from edge to person
		4. apply normal force to actor

		TODO: for now we will assume that the bounding box is oriented vertically or horizontally 
		*/
	
		
		ignition::math::Vector3d modelPos = model->WorldPose().Pos(); // position of model
		modelPos.Z() = 0;
		auto actorPos = actorPose.Pos(); // position of actor
		actorPos.Z() = 0;
	
		
		ignition::math::Box box = model->BoundingBox();
		ignition::math::Vector3d min_corner = box.Min();
		ignition::math::Vector3d max_corner = box.Max();
		min_corner.Z() = 0;
		max_corner.Z() = 0;


		//TODO: ensure that these methods work using Line3d
		ignition::math::Line3d left = ignition::math::Line3d(min_corner.X(),min_corner.Y(),min_corner.X(), max_corner.Y());
		ignition::math::Line3d right = ignition::math::Line3d(max_corner.X(),min_corner.Y(),max_corner.X(), max_corner.Y());
		ignition::math::Line3d top = ignition::math::Line3d(min_corner.X(),max_corner.Y(),max_corner.X(), max_corner.Y());
		ignition::math::Line3d bot = ignition::math::Line3d(min_corner.X(),min_corner.Y(),max_corner.X(), min_corner.Y());
		
		std::vector<ignition::math::Line3d> edges = {left, right, top, bot};

		ignition::math::Vector3d min_normal;
		double min_mag = 1000000;
		bool found = false;

		//printf("pos: (%f, %f, %f)\t", actorPose.Pos().X(), actorPose.Pos().Y(), actorPose.Pos().Z());

		for (ignition::math::Line3d edge: edges){

			//printf("edge: (%f,%f)->(%f,%f)\t", edge[0].X(), edge[0].Y(), edge[1].X(), edge[1].Y());

			ignition::math::Vector3d edge_vector = edge.Direction(); // vector in direction of edge 
			
			ignition::math::Vector3d pos_vector = ignition::math::Vector3d(actorPose.Pos().X()-edge[0].X(), actorPose.Pos().Y()-edge[0].Y(), 0);// vector from edge corner to actor pos
			
			ignition::math::Vector3d proj = ((pos_vector.Dot(edge_vector))/(edge_vector.Dot(edge_vector)))*edge_vector; // project pos_vector onto edge_vector
			/*
			std::printf("pos_v (%f, %f, %f)\t", pos_vector.X(), pos_vector.Y(), pos_vector.Z());
			std::printf("edge_v (%f, %f, %f)\t", edge_vector.X(), edge_vector.Y(), edge_vector.Z());
			std::printf("proj_v (%f, %f, %f)\n", proj.X(),proj.Y(), proj.Z());
			*/
			//check if the projected point is within the edge
			if (edge.Within(proj+edge[0])){
				//compute normal
				ignition::math::Vector3d normal = pos_vector-proj;
				//std::printf("in here\n");
				if (normal.Length() < min_mag){
					min_normal = normal;
					min_mag = normal.Length();
					found = true;
				}

			}
		
		}
		
		
		// if conditions are met for this edge (and normal): boundary_force += normal/(dist*dist);
		double dist = min_normal.Length();
		if (found && dist < this->dataPtr->obstacleMargin){
			min_normal.Normalize();
			boundary_force += min_normal/(dist*dist);
		}

	}
	//std::printf("(%f, %f, %f)\n", boundary_force.X(),boundary_force.Y(), boundary_force.Z());
	return boundary_force;

}

ignition::math::Vector3d ActorPlugin::TargetForce(){
	/*
	return a steering force towards the desired target
	*/
	ignition::math::Vector3d steer = ignition::math::Vector3d(0,0,0);
	ignition::math::Vector3d actorPos = ignition::math::Vector3d(this->dataPtr->actor->WorldPose().Pos().X(), this->dataPtr->actor->WorldPose().Pos().Y(), 0);

	ignition::math::Vector3d desired = this->dataPtr->curr_target - actorPos;
	desired.Normalize();
	desired*=this->dataPtr->max_speed;

	steer = desired - this->dataPtr->velocity;

	if (steer.Length() > this->dataPtr->max_force){
		steer.Normalize();
		steer*=this->dataPtr->max_force;
	}

	
	return steer;
}

void ActorPlugin::SelectRandomTarget(){
	/*
	Set curr_target equal to a random target in bounds, store prev_target
	*/
	ignition::math::Pose3d actorPose = this->dataPtr->actor->WorldPose();
	/*
	cast a ray in a random direction.
	store the closest intersection with a boundary or bounding box
	choose a random point along the resultant ray as the new target
	*/

	ignition::math::Vector3d actorPos = actorPose.Pos();
	actorPos.Z() = 0;
	ignition::math::Vector3d new_target = this->dataPtr->curr_target;
	bool target_found = false;

	auto world = this->dataPtr->actor->GetWorld();

	while (!target_found){
		//chose a random new direction (rotated from current direction) to cast the ray:
		//ignition::math::Vector3d ray_dir = ignition::math::Vector3d(ignition::math::Rand::DblUniform())
		ignition::math::Vector3d dir = this->dataPtr->velocity;
		if (dir.Length() < 1e-6){
			dir = ignition::math::Vector3d(ignition::math::Rand::DblUniform(-1, 1),ignition::math::Rand::DblUniform(-1, 1),0);
		}
		dir.Normalize();
		ignition::math::Quaterniond rotation =  ignition::math::Quaterniond::EulerToQuaternion(0,0,ignition::math::Rand::DblUniform(-3, 3)); 
		dir = rotation.RotateVector(dir);
		//make direction vector large and add it to current position to obtain ray line 
		dir*=10000;
		ignition::math::Line3d ray = ignition::math::Line3d(actorPos.X(), actorPos.Y(), actorPos.X() + dir.X(), actorPos.Y() + dir.Y());

		ignition::math::Vector3d closest_intersection;
		double min_dist = 100000;

		bool flag = false;

		//iterate over all boundaries:
		/*
		for (int i =0; i<(int) this->dataPtr->polygon.size(); i+=2){
			ignition::math::Line3d boundary_line = ignition::math::Line3d(this->dataPtr->polygon[i].X(), this->dataPtr->polygon[i].Y(), this->dataPtr->polygon[i+1].X(), this->dataPtr->polygon[i+1].Y());
			
			
			ignition::math::Vector3d test_intersection;

			if (ray.Intersect(boundary_line, test_intersection)){ //if the ray intersects the boundary
				//std::printf("intersection: (%f, %f, %f)\n", test_intersection.X(), test_intersection.Y(), test_intersection.Z());
				double dist_to_int = (test_intersection-actorPos).Length();
				if (dist_to_int < min_dist){
					min_dist = dist_to_int;
					closest_intersection = test_intersection;
				}

			}
		}
		*/

		//iterate over all objects

		for (unsigned int i = 0; i < world->ModelCount(); ++i) {
			// iterate over all models. Skip if the model is itself or if it needs to be ignored 
			
			auto model = world->ModelByIndex(i);
			auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
			
			if (act) {
				continue;
			}

			/*
			1. obtain bouding box of model
			2. check ray intersection with all edge lines 
			3. store the closest intersection
			*/
		
			
			
			
			if (model->GetName() == "myhal" || model->GetName() == "test_cell2"){
				for (auto link: this->dataPtr->building_links){


					ignition::math::Vector3d modelPos = link->WorldPose().Pos(); // position of model
					modelPos.Z() = 0;
		
					if (modelPos.Z() > 1.5){ 
						continue;
					}

					ignition::math::Box box = link->BoundingBox();
					ignition::math::Vector3d min_corner = box.Min();
					ignition::math::Vector3d max_corner = box.Max();
					min_corner.Z() = 0;
					max_corner.Z() = 0;


					//TODO: ensure that these methods work using Line3d
					ignition::math::Line3d left = ignition::math::Line3d(min_corner.X(),min_corner.Y(),min_corner.X(), max_corner.Y());
					ignition::math::Line3d right = ignition::math::Line3d(max_corner.X(),min_corner.Y(),max_corner.X(), max_corner.Y());
					ignition::math::Line3d top = ignition::math::Line3d(min_corner.X(),max_corner.Y(),max_corner.X(), max_corner.Y());
					ignition::math::Line3d bot = ignition::math::Line3d(min_corner.X(),min_corner.Y(),max_corner.X(), min_corner.Y());
					
					std::vector<ignition::math::Line3d> edges = {left, right, top, bot};

					//iterate over all edges and test for intersection
					for (ignition::math::Line3d edge: edges){
						ignition::math::Vector3d test_intersection;

						if (ray.Intersect(edge, test_intersection)){ //if the ray intersects the boundary
							double dist_to_int = (test_intersection-actorPos).Length();
							if (dist_to_int < min_dist){
								min_dist = dist_to_int;
								closest_intersection = test_intersection;
							}

						}
						
					
					}
				}
			} else{

				ignition::math::Vector3d modelPos = model->WorldPose().Pos(); // position of model
				modelPos.Z() = 0;
				ignition::math::Box box = model->BoundingBox();
				ignition::math::Vector3d min_corner = box.Min();
				ignition::math::Vector3d max_corner = box.Max();
				min_corner.Z() = 0;
				max_corner.Z() = 0;


				//TODO: ensure that these methods work using Line3d
				ignition::math::Line3d left = ignition::math::Line3d(min_corner.X(),min_corner.Y(),min_corner.X(), max_corner.Y());
				ignition::math::Line3d right = ignition::math::Line3d(max_corner.X(),min_corner.Y(),max_corner.X(), max_corner.Y());
				ignition::math::Line3d top = ignition::math::Line3d(min_corner.X(),max_corner.Y(),max_corner.X(), max_corner.Y());
				ignition::math::Line3d bot = ignition::math::Line3d(min_corner.X(),min_corner.Y(),max_corner.X(), min_corner.Y());
				
				std::vector<ignition::math::Line3d> edges = {left, right, top, bot};

				//iterate over all edges and test for intersection
				for (ignition::math::Line3d edge: edges){
					ignition::math::Vector3d test_intersection;

					if (ray.Intersect(edge, test_intersection)){ //if the ray intersects the boundary
						double dist_to_int = (test_intersection-actorPos).Length();
						if (dist_to_int < min_dist){
							min_dist = dist_to_int;
							closest_intersection = test_intersection;
						}

					}
					
				
				}

			}
		
		}

		//now we have our intersection point 

		ignition::math::Vector3d final_ray = closest_intersection - actorPos;
		
		
		//randomly scale the ray 

		auto v_to_add = final_ray*ignition::math::Rand::DblUniform(0.3,0.9);


		//TODO: fix this "too close check" with a projection
		if ((final_ray-v_to_add).Length() < (this->dataPtr->obstacleMargin)){ // if the target is too close
			v_to_add.Normalize();
			auto small_subtraction = (v_to_add*this->dataPtr->obstacleMargin)*2;
			v_to_add = final_ray - small_subtraction;
			if (small_subtraction.Length() > final_ray.Length()){
				v_to_add*=0;
			}
		}

		//add the vector to our current position to obtain the new target pos
		
		new_target = v_to_add + actorPos;
		target_found = true;
	}

	//std::printf("(%f, %f, %f)\n", new_target.X(), new_target.Y(), new_target.Z());
	this->dataPtr->prev_target = this->dataPtr->curr_target;
	this->dataPtr->curr_target = new_target;
}

void ActorPlugin::NetForceUpdate(){


	auto actor = this->ActorAvoidance();
	if (actor.Length() > 1e-6){
		this->SelectRandomTarget();
	}
	//auto boundary = this->WithinBounds();
	auto target = this->TargetForce();
	//std::printf("actor (%f, %f, %f) \n", actor.X(), actor.Y(), actor.Z());
	//std::printf("target (%f, %f, %f) \n", target.X(), target.Y(), target.Z()); 

	this->dataPtr->F_net = actor + target;// + boundary;

	
}

void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
	// Time delta
	this->dataPtr->dt = (_info.simTime - this->dataPtr->lastUpdate).Double();

	if (this->dataPtr->dt < 1/this->dataPtr->updateFreq)
		return;
	

	this->dataPtr->lastUpdate = _info.simTime;

	ignition::math::Pose3d actorPose = this->dataPtr->actor->WorldPose();

	// retarget iff we are close to the current target or a certain number of second have elapsed since last retarget
	auto pos = actorPose.Pos();
	pos.Z() = 0;
	if ((pos - this->dataPtr->curr_target).Length() < this->dataPtr->targetRadius || (_info.simTime- this->dataPtr->last_target_time).Double() > 10){
		this->SelectRandomTarget();
		this->dataPtr->last_target_time = _info.simTime;
	}
	

	
	this->NetForceUpdate();

	//Given a force: we will update the velocity, position and acceleration of the actor 
	
	
	
   
	ignition::math::Vector3d acceleration = this->dataPtr->F_net/this->dataPtr->mass;
   
	this->dataPtr->velocity += this->dataPtr->dt*acceleration;
	if (this->dataPtr->velocity.Length() > this->dataPtr->max_speed){
		this->dataPtr->velocity.Normalize();
		this->dataPtr->velocity *= this->dataPtr->max_speed;
	}
   
	ignition::math::Vector3d dir = this->dataPtr->velocity;
	dir.Normalize();
   
   
	double currentYaw = actorPose.Rot().Yaw();
	ignition::math::Angle yawDiff = atan2(dir.Y(), dir.X()) + IGN_PI_2 - currentYaw;
	yawDiff.Normalize();
	
	
	actorPose.Pos() += this->dataPtr->velocity * this->dataPtr->dt;
	actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, currentYaw + yawDiff.Radian()*0.1);
	
	
	// Distance traveled is used to coordinate motion with the walking
	// animation
	double distanceTraveled = (actorPose.Pos() - this->dataPtr->actor->WorldPose().Pos()).Length();

	// Update actor
	//this->dataPtr->actor->FillMsg(msgs::Convert(this->dataPtr->velocity));	
	
	this->dataPtr->actor->SetWorldPose(actorPose, false, false);
	this->dataPtr->actor->SetScriptTime(this->dataPtr->actor->ScriptTime() + (distanceTraveled * this->dataPtr->animationFactor));

	
	//this->dataPtr->actor->SetLinearVel(ignition::math::Vector3d(1,0,0));
	

}

