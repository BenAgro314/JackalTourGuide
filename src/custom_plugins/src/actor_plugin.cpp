
///TODO: RETARGET AFTER A CERTAIN AMOUNT OF TIME 

#include <functional>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Line2.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Box.hh>
#include <vector>
#include <string>
#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>

#include "actor_plugin.hh"

using namespace gazebo;
using namespace servicesim;

GZ_REGISTER_MODEL_PLUGIN(servicesim::ActorPlugin)

class servicesim::ActorPluginPrivate
{
  /// \brief Pointer to the actor.
  public: physics::ActorPtr actor{nullptr};

  /// \brief Velocity of the actor
  public: double velocity{0.8};
  
  public: double curr_vel{0.8};

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

  public: double initial_z;
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
  if (_sdf->HasElement("velocity"))
    this->dataPtr->velocity = _sdf->Get<double>("velocity");
    this->dataPtr->curr_vel = this->dataPtr->velocity;

 
	auto pointElem = _sdf->GetElement("point");
	while (pointElem){
		
		this->dataPtr->polygon.push_back(pointElem->Get<ignition::math::Vector2d>());
		pointElem = pointElem->GetNextElement("point");
	}
	
	/*
	for (int i =0; i< (int) this->dataPtr->polygon.size(); ++i){
		std::printf("(%f, %f)\n", this->dataPtr->polygon[i].X(), this->dataPtr->polygon[i].Y());
	}
	*/
	   
	   //read in starting pose
	   
	   if (_sdf->HasElement("pose")){
		    this->dataPtr->prev_target = _sdf->GetElement("pose")->Get<ignition::math::Pose3d>();
	   } 
		this->dataPtr->curr_target = this->dataPtr->prev_target;
		this->dataPtr->initial_z = this->dataPtr->prev_target.Pos().Z();

  if (_sdf->HasElement("follower")){
	  this->dataPtr->follower = _sdf->Get<bool>("follower");
  } else{
	  this->dataPtr->follower = false;
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
  
 
}


ignition::math::Vector3d ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos, const common::UpdateInfo &_info)
{		
	  auto actorWorld = ignition::math::Matrix4d(this->dataPtr->actor->WorldPose());
	  auto world = this->dataPtr->actor->GetWorld();
	  
	  auto res = ignition::math::Vector3d(0,0,0);
	  
	  // Iterate over all models in the world
	  for (unsigned int i = 0; i < world->ModelCount(); ++i)
	  {
		// Skip if it's not an obstacle
		// Fixme: automatically adding all actors to obstacles
		auto model = world->ModelByIndex(i);
		auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
		if ((model->GetName() == "myhal" || model->GetName() == "ground_plane"))
		{
		  continue;
		}
		if (act && act == this->dataPtr->actor)
		  continue;
		  
		  
		auto modelWorld = ignition::math::Matrix4d(model->WorldPose());

		// Model in actor's frame
		auto modelActor = actorWorld.Inverse() * modelWorld;
		
	 
		ignition::math::Vector3d offset = model->WorldPose().Pos() - this->dataPtr->actor->WorldPose().Pos();
		offset.Z() = 0;
		double modelDist = offset.Length();
      
		
		if (offset.Length() < this->dataPtr->obstacleMargin) //&& std::abs(modelActor(0, 3)) < this->dataPtr->obstacleMargin* 0.4 && modelActor(2, 3) > 0)
		{
			 
			 
	
			double theta = IGN_DTOR(180);
			
			
			double invModelDist = 1 / modelDist;
			offset.Normalize();
			offset *= invModelDist;

			// rotate offset by theta degrees about z axis (in the xy plane);
			
			ignition::math::Matrix3d rot_matrix = ignition::math::Matrix3d(std::cos(theta), -1*std::sin(theta), 0, std::sin(theta), std::cos(theta), 0, 0,0,1);
			offset = rot_matrix*offset;
			
			
			res+=offset;
			
			
			this->dataPtr->targetPose = this->RandomTarget(_info);
		
			
		} 
		if (offset.Length() < (this->dataPtr->obstacleMargin)*2 && std::abs(modelActor(0, 3)) < this->dataPtr->obstacleMargin * 0.4 && modelActor(2, 3) > 0)
		{	
	
			this->dataPtr->curr_vel = 0.7; //add scaling to this
			
		}

		}
		return res;
}




ignition::math::Pose3d ActorPlugin::RandomTarget(const common::UpdateInfo &_info){
	
	auto actorPose =this->dataPtr->actor->WorldPose();
	auto actorPos = actorPose.Pos();
	double currentYaw = actorPose.Rot().Yaw();
	
	auto posDiff = this->dataPtr->curr_target.Pos() - actorPos;
	posDiff.Z(0);
	
	double distance = posDiff.Length();

    // Still far from target or havent reached target in a while?
	if ((distance > this->dataPtr->targetRadius) && (_info.simTime - this->dataPtr->last_target_time).Double() <10){
		return this->dataPtr->curr_target;
	}
	
	bool not_suitable = true;
	
	auto world = this->dataPtr->actor->GetWorld();
	ignition::math::Vector2d C;
	
	
	while (not_suitable){
		//put random point selection from polygon in here
		/*
		 * Steps:
		 * 1. choose random ray direction from current position (point A), within some angle from where we are currently facing 
		 * 2. extend ray until it hits the edge of the polygon (point B)
		 * 3. choose random point along the segment AB and set this as the test target (call this point C)
		 * 4. check if this is too close to the other models in the world, if it is, choose again
		 * 
		 */
		
		
		double angle_shift = ignition::math::Rand::DblUniform(-2, 2); 
		double angle = currentYaw+angle_shift;// choose a random new angle from within 90 degrees of where we are currently looking 
		
		double slope = std::tan(angle);
		double max_radius = 100;
		
		
		double bx,by;
		
		bool increase = (bool) ignition::math::Rand::IntUniform(0,1);
		
		if (increase){
			bx = actorPos.X() + max_radius/(std::sqrt(1+(slope*slope)));
			by = actorPos.Y() + (max_radius*slope)/(std::sqrt(1+(slope*slope)));
		} else{
			bx = actorPos.X() - max_radius/(std::sqrt(1+(slope*slope)));
			by = actorPos.Y() - (max_radius*slope)/(std::sqrt(1+(slope*slope)));
		}
		
		//std::printf("far: (%f, %f) ", bx, by);
		ignition::math::Line2d ray = ignition::math::Line2d(actorPos.X(), actorPos.Y(), bx, by);
		bool flag = false;
		
		// iterate over all line segments in the polygon and check for min distance intersection
		ignition::math::Vector2d B;
		double min_distance = 10000;
		
		for (int i =0; i<(int) this->dataPtr->polygon.size(); i+=2){
			ignition::math::Line2d test_line = ignition::math::Line2d(this->dataPtr->polygon[i], this->dataPtr->polygon[i+1]);
			//std::printf("test_line length: %f \n", test_line.Length());
			// check for intersection between the ray and the testline
			
			ignition::math::Vector2d test_point;
			if (std::intersect(test_line,ray,test_point)){
				ignition::math::Line2d test_seg = ignition::math::Line2d(actorPos.X(), actorPos.Y(), test_point.X(), test_point.Y());
				if (test_seg.Length() < min_distance){
					B = test_point;
					min_distance = test_seg.Length();
				}
				flag = true; 
			}
		}
		if (!flag){
			B = ignition::math::Vector2d(actorPos.X(), actorPos.Y());
		}
		//std::printf("B: (%f, %f)\n", B.X(), B.Y());
		
		ignition::math::Line2d seg = ignition::math::Line2d(actorPos.X(), actorPos.Y(), B.X(), B.Y());
		//std::printf("seg length: %f, actorPos (%f, %f), B: (%f, %f)\n", seg.Length(),actorPos.X(), actorPos.Y(),B.X(), B.Y()); 
		double length = seg.Length();
		double rand_length = ignition::math::Rand::DblUniform(0,length)*0.95;
		
		
		if (increase){
			C.X() = actorPos.X() + rand_length/(std::sqrt(1+(slope*slope)));
			C.Y() = actorPos.Y() + (rand_length*slope)/(std::sqrt(1+(slope*slope)));
		
		} else{
			C.X() = actorPos.X() - rand_length/(std::sqrt(1+(slope*slope)));
			C.Y() = actorPos.Y() - (rand_length*slope)/(std::sqrt(1+(slope*slope)));
		}
		
		/// TODO: ADD SAFETY CHECK FOR C
		/*
		 * 1. Iterate over all models in the scene (do not consider myhal or groundplane
		 * 2. check if there is a model in the way of the target
		 * 3. if it does, rechoose target, else the target is suitable
		 */
		auto world = this->dataPtr->actor->GetWorld();
		not_suitable = false;
		for (unsigned int i = 0; i < world->ModelCount(); ++i){
			
			auto model = world->ModelByIndex(i);
			auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
			
			if ((model->GetName() == "myhal") || (model->GetName() == "ground_plane") || (act && act == this->dataPtr->actor)){
			  continue;
			}
			
			if (!act){ 
				// test if the target pose falls within the bounding box of the model
				ignition::math::Box bound_box = model->BoundingBox();
				ignition::math::Vector3d min_corner = bound_box.Min();
				ignition::math::Vector3d max_corner = bound_box.Max();
				
				// we will expand the bounds a little bit
				double min_x = std::min(min_corner.X(),max_corner.X())-0.3;
				double max_x = std::max(min_corner.X(),max_corner.X())+0.3;
				double min_y = std::min(min_corner.Y(),max_corner.Y())-0.3;
				double max_y = std::max(min_corner.Y(),max_corner.Y())+0.3;
				//std::printf("(%f, %f, %f, %f)\n", min_x, max_x, min_y, max_y);
				//CHECK RAY INTERSECTION WITH BOUNDING BOX
				
				ignition::math::Line2d left = ignition::math::Line2d(min_x,min_y, min_x, max_y);
				ignition::math::Line2d right = ignition::math::Line2d(max_x,min_y, max_x, max_y);
				ignition::math::Line2d bot = ignition::math::Line2d(min_x,min_y, max_x, min_y);
				ignition::math::Line2d top = ignition::math::Line2d(min_x,max_y, max_x, max_y);
				
				ignition::math::Line2d target_ray = ignition::math::Line2d(actorPos.X(), actorPos.Y(), C.X(), C.Y());
				
				
				ignition::math::Vector2d test_point;
				ignition::math::Vector2d holder;
				
				if (std::intersect(target_ray, left, holder)){// && target_ray.OnSegment(test_point)){
					//std::printf("intersection1\n");
					not_suitable = true;
					break;
				}
				if (std::intersect(target_ray, right, holder)){// && target_ray.OnSegment(test_point)){
					//std::printf("intersection1\n");
					not_suitable = true;
					break;
				}
				if (std::intersect(target_ray, top, holder)){// && target_ray.OnSegment(test_point)){
					//std::printf("intersection1\n");
					not_suitable = true;
					break;
				}
				if (std::intersect(target_ray, bot, holder)){// && target_ray.OnSegment(test_point)){
					//std::printf("intersection1\n");
					not_suitable = true;
					break;
				}
				
			
			} else{
				double buffer = this->dataPtr->obstacleMargin*2;

				ignition::math::Vector3d object_pos = model->WorldPose().Pos();
				object_pos.Z() = 0;
			
				double dist = (object_pos -  ignition::math::Vector3d(C.X(),C.Y(),0)).Length();
			  
			  
				if (dist < buffer) {
					not_suitable = true;
				}
				
			}
		}

	}


	ignition::math::Vector3d dir = ignition::math::Vector3d(C.X()-this->dataPtr->prev_target.Pos().X(), C.Y()-this->dataPtr->prev_target.Pos().Y(), 0);
	double yaw = std::atan2(dir.Y(), dir.X());
	ignition::math::Pose3d res = ignition::math::Pose3d(C.X(),C.Y(),this->dataPtr->initial_z,1.5708,0,yaw);
	
	this->dataPtr->prev_target = this->dataPtr->curr_target;
	this->dataPtr->curr_target = res;
	
	this->dataPtr->last_target_time = _info.simTime;
	return res;
}


void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->dataPtr->lastUpdate).Double();

  if (dt < 1/this->dataPtr->updateFreq)
    return;


  
  this->dataPtr->lastUpdate = _info.simTime;

    
  // Current pose - actor is oriented Y-up and Z-front
  auto actorPose = this->dataPtr->actor->WorldPose();

  // Targeting 
  //select random target in bounds 
	

  this->dataPtr->targetPose = this->RandomTarget(_info);
 
  // Direction to target
  auto dir = (this->dataPtr->targetPose.Pos() - actorPose.Pos()).Normalize();
  
  
  //attempting to handle obsticles
  this->dataPtr->curr_vel = this->dataPtr->velocity; 
  ignition::math::Vector3d shift = this->HandleObstacles(dir,_info);
  actorPose.Pos() += shift * this->dataPtr->curr_vel * dt;


  // TODO: generalize for actors facing other directions
  auto currentYaw = actorPose.Rot().Yaw();

  // Difference to target
  ignition::math::Angle yawDiff = atan2(dir.Y(), dir.X()) + IGN_PI_2 - currentYaw;
  yawDiff.Normalize();


  actorPose.Pos() += dir * this->dataPtr->curr_vel * dt;
  actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, currentYaw + yawDiff.Radian()*0.1);


  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() - this->dataPtr->actor->WorldPose().Pos()).Length();

  // Update actor
  this->dataPtr->actor->SetWorldPose(actorPose, false, false);
  this->dataPtr->actor->SetScriptTime(this->dataPtr->actor->ScriptTime() +
    (distanceTraveled * this->dataPtr->animationFactor));
    

}

