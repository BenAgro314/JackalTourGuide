
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

#include "boids_plugin.hh"

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
  
  public: ignition::math::Vector3d velocity = ignition::math::Vector3d(1,0,0);

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
  
  public: ignition::math::Pose3d prev_target;
  
  public: ignition::math::Pose3d curr_target;
  
  public: ignition::math::Pose3d targetPose;
  
  public: common::Time last_target_time;
  
  public: bool follower = false;
  
  public: double n_dist = 2;
  
  public: double dt;
  
  public: std::map<std::string, ignition::math::Vector3d> prev_poses;
  
  public: double cohesion_factor = 0.01;
  public: double alignment_factor = 0.1;
  public: double aversion_factor = 1;

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
	   
	   if (_sdf->HasElement("pose")){
		    this->dataPtr->prev_target = _sdf->GetElement("pose")->Get<ignition::math::Pose3d>();
	   } 
		this->dataPtr->curr_target = this->dataPtr->prev_target;
  if (_sdf->HasElement("follower")){
	  this->dataPtr->follower = _sdf->Get<bool>("follower");
  } else{
	  this->dataPtr->follower = false;
  }
  
	if (_sdf->HasElement("velocity")){
		this->dataPtr->velocity = _sdf->GetElement("velocity")->Get<ignition::math::Vector3d>();
	} 
	
	if (_sdf->HasElement("aversion_factor")){
		this->dataPtr->aversion_factor = _sdf->GetElement("aversion_factor")->Get<double>();
	} 
	
	if (_sdf->HasElement("alignment_factor")){
		this->dataPtr->alignment_factor = _sdf->GetElement("alignment_factor")->Get<double>();
	} 
	
	if (_sdf->HasElement("cohesion_factor")){
		this->dataPtr->cohesion_factor = _sdf->GetElement("cohesion_factor")->Get<double>();
	} 
	
	if (_sdf->HasElement("n_dist")){
		this->dataPtr->n_dist = _sdf->GetElement("n_dist")->Get<double>();
	} 
  

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
		
		if (!act) {
			continue;
		} if (act && act == this->dataPtr->actor){
			continue;
		}

		
		
		ignition::math::Vector3d modelPos = model->WorldPose().Pos();
		modelPos.Z() = 0;
		this->dataPtr->prev_poses[model->GetName()] = modelPos;
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


ignition::math::Vector3d ActorPlugin::BoidAvoidance(){
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
		
		if (act || (model->GetName() == "ground_plane")) {
			continue;
		}

		/*
		1. obtain bouding box of model
		2. for each edge of bouning box, find "the one we are closet too?" (the one with the closest perp distance and with a normal that intersects the edge)
		3. find normal pointing away from edge to person
		4. apply normal force to actor

		TODO: for now we will assume that the bounding box is oriented vertically or horizontally 
		*/

		//create vector of models links:

	
	
		if (model->GetName() == "myhal" || model->GetName() == "test_cell2"){ //add building names here TODO: macro for building names 
			//std::cout << "in here\n";
			//TODO: find ways of speeding this up

			auto actorPos = actorPose.Pos(); // position of actor
			actorPos.Z() = 0;

			for (auto link: this->dataPtr->building_links){
				//std::cout << "in here\n";
				
				ignition::math::Vector3d modelPos = link->WorldPose().Pos(); // position of model
				modelPos.Z() = 0;
		
				if ((actorPos-modelPos).Length() > 5){ //this should eleminate many links from consideration
					continue;
				}

				//ignition::math::Vector3d rad = actorPos-modelPos;
				//double dist = rad.Length();
				
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

				ignition::math::Vector3d min_normal;
				double min_mag = 1000000;
				bool found = false;

				//printf("pos: (%f, %f, %f)\t", actorPose.Pos().X(), actorPose.Pos().Y(), actorPose.Pos().Z());

				for (ignition::math::Line3d edge: edges){

					//printf("edge: (%f,%f)->(%f,%f)\t", edge[0].X(), edge[0].Y(), edge[1].X(), edge[1].Y());

					ignition::math::Vector3d edge_vector = edge.Direction(); // vector in direction of edge 
					
					ignition::math::Vector3d pos_vector = ignition::math::Vector3d(actorPose.Pos().X()-edge[0].X(), actorPose.Pos().Y()-edge[0].Y(), 0);// vector from edge corner to actor pos
					
					ignition::math::Vector3d proj = ((pos_vector.Dot(edge_vector))/(edge_vector.Dot(edge_vector)))*edge_vector; // project pos_vector onto edge_vector
			
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
		} else{
				ignition::math::Vector3d modelPos = model->WorldPose().Pos(); // position of model
				modelPos.Z() = 0;
				auto actorPos = actorPose.Pos(); // position of actor
				actorPos.Z() = 0;
				//ignition::math::Vector3d rad = actorPos-modelPos;
				//double dist = rad.Length();
				
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

	}
	//std::printf("(%f, %f, %f)\n", boundary_force.X(),boundary_force.Y(), boundary_force.Z());
	return boundary_force;

}

//alignement: find average velocity of neighbours 

ignition::math::Vector3d  ActorPlugin::Alignment(){
	
	ignition::math::Pose3d actorPose = this->dataPtr->actor->WorldPose();
	
	ignition::math::Vector3d sum = ignition::math::Vector3d(0,0,0);
	
	auto world = this->dataPtr->actor->GetWorld();
	
	
	int count = 0;
	for (unsigned int i = 0; i < world->ModelCount(); ++i) {
		// iterate over all models. Skip if the model is itself or not an actor
		
		auto model = world->ModelByIndex(i);
		auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
		
		if (!act) {
			continue;
		} if (act && act == this->dataPtr->actor){
			continue;
		}
		
		ignition::math::Vector3d otherPos = model->WorldPose().Pos();
		otherPos.Z() = 0;
		auto actorPos = actorPose.Pos();
		actorPos.Z() = 0;
		
		if ((actorPos-otherPos).Length() < this->dataPtr->n_dist){
			//if we are in the radius of consideration
			
			ignition::math::Vector3d past_pos = this->dataPtr->prev_poses[model->GetName()];
			this->dataPtr->prev_poses[model->GetName()] = otherPos;
			
			ignition::math::Vector3d vel = (otherPos-past_pos)/this->dataPtr->dt;
			count++;
			//ignition::math::Vector3d vel = model->WorldLinearVel();	
			sum+= vel;
			//std::printf("(%f, %f, %f)\n", vel.X(), vel.Y(), vel.Z());
		}
		
	}
	
	if (count >0){
		
		// Implement Reynolds: Steering = Desired - Velocity
		sum.Normalize();
		sum*=this->dataPtr->max_speed;
		sum-=this->dataPtr->velocity;
		if (sum.Length() > this->dataPtr->max_force){
			sum.Normalize();
			sum*=this->dataPtr->max_force;
		}
	}
	
	return sum;
	
}

//steer towards average position of nearby boids
ignition::math::Vector3d  ActorPlugin::Cohesion(){
	ignition::math::Pose3d actorPose = this->dataPtr->actor->WorldPose();
	
	ignition::math::Vector3d sum = ignition::math::Vector3d(0,0,0);
	
	auto world = this->dataPtr->actor->GetWorld();
	
	
	int count = 0;
	for (unsigned int i = 0; i < world->ModelCount(); ++i) {
		// iterate over all models. Skip if the model is itself or not an actor
		
		auto model = world->ModelByIndex(i);
		auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
		
		if (!act) {
			continue;
		} if (act && act == this->dataPtr->actor){
			continue;
		}
		
		ignition::math::Vector3d otherPos = model->WorldPose().Pos();
		otherPos.Z() = 0;
		auto actorPos = actorPose.Pos();
		actorPos.Z() = 0;
		
		if ((actorPos-otherPos).Length() < this->dataPtr->n_dist){
			//if we are in the radius of consideration
			count++;
			sum+= otherPos;
			
		}
		
	}
	
	if (count >0){
		auto actorPos = actorPose.Pos();
		actorPos.Z() = 0;
		sum = sum - actorPos;
		
		// Implement Reynolds: Steering = Desired - Velocity
		sum.Normalize();
		sum*=this->dataPtr->max_speed;
		sum-=this->dataPtr->velocity;
		
		
		if (sum.Length() > this->dataPtr->max_force){
			sum.Normalize();
			sum*=this->dataPtr->max_force;
		}
	}
	//std::printf("(%f, %f, %f)\n", sum.X(), sum.Y(), sum.Z());
	return sum;
	
}


void ActorPlugin::NetForceUpdate(){
	
	
	//this->dataPtr->F_net = ignition::math::Vector3d(20,0,0);
	
	
	ignition::math::Pose3d actorPose = this->dataPtr->actor->WorldPose();
	//this->Alignement();
	
	auto obstacle= this->ObstacleAvoidance();
	//auto boundary = this->WithinBounds();
	auto boid = this->BoidAvoidance();
	auto cohesion = this->Cohesion();
	auto alignment = this->Alignment();
	
	
	auto dir = this->dataPtr->velocity;
	dir.Normalize();

	this->dataPtr->F_net = obstacle
	+(this->dataPtr->aversion_factor*boid)
	+(this->dataPtr->cohesion_factor*cohesion)
	+(this->dataPtr->alignment_factor*alignment);

	//add a "walking" force so the boids don't slow down too much
	/*
	if (this->dataPtr->velocity.Length() < (this->dataPtr->max_speed)*0.7){
		this->dataPtr->F_net+=dir*this->dataPtr->max_force*(((0.7*this->dataPtr->max_speed)-this->dataPtr->velocity.Length())/(0.7*this->dataPtr->max_speed));
	}
	*/
	
}

void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
	// Time delta
	this->dataPtr->dt = (_info.simTime - this->dataPtr->lastUpdate).Double();

	if (this->dataPtr->dt < 1/this->dataPtr->updateFreq)
		return;
	

	this->dataPtr->lastUpdate = _info.simTime;
	
	this->NetForceUpdate();
  
	//find F_net up hear
  
	
	//Given a force: we will update the velocity, position and acceleration of the actor 
	
	ignition::math::Pose3d actorPose = this->dataPtr->actor->WorldPose();
	
   
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

