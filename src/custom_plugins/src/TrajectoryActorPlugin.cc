/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>

#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>

#include "TrajectoryActorPlugin.hh"

using namespace gazebo;
using namespace servicesim;
GZ_REGISTER_MODEL_PLUGIN(servicesim::TrajectoryActorPlugin)

class servicesim::TrajectoryActorPluginPrivate
{
  /// \brief Pointer to the actor.
  public: physics::ActorPtr actor{nullptr};

  /// \brief Velocity of the actor
  public: double velocity{0.8};
  
  public: double curr_vel{0.8};

  /// \brief List of connections such as WorldUpdateBegin
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief List of targets
  public: std::vector<ignition::math::Pose3d> targets;

  /// \brief Current target index
  public: int currentTarget{0};

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

  /// \brief Time when the corner starts
  public: common::Time firstCornerUpdate;
  
      
      
  public: common::Time lastRetarget;

  /// \brief List of models to avoid
  public: std::vector<std::string> obstacles;

  /// \brief Animation for corners
  public: common::PoseAnimation *cornerAnimation{nullptr};

  /// \brief Frequency in Hz to update
  public: double updateFreq{60};
	
  public: bool random_walk = false;
  
  public: ignition::math::Vector4d boundaries;
  
  public: ignition::math::Pose3d prev_target;
  
  public: ignition::math::Pose3d curr_target;
  
  public: ignition::math::Pose3d targetPose;
};

/////////////////////////////////////////////////
TrajectoryActorPlugin::TrajectoryActorPlugin()
    : dataPtr(new TrajectoryActorPluginPrivate)
{
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&TrajectoryActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Update frequency
  if (_sdf->HasElement("update_frequency"))
    this->dataPtr->updateFreq = _sdf->Get<double>("update_frequency");

  // Read in the velocity
  if (_sdf->HasElement("velocity"))
    this->dataPtr->velocity = _sdf->Get<double>("velocity");
    this->dataPtr->curr_vel = this->dataPtr->velocity;
    
  if (_sdf->HasElement("random")){
	  this->dataPtr->random_walk = _sdf->Get<bool>("random");
  } else{
	  this->dataPtr->random_walk = false;
	 }

  if (!this->dataPtr->random_walk){
	  // Read in the target poses
	 
	  auto targetElem = _sdf->GetElement("target");
	  while (targetElem)
	  {
		this->dataPtr->targets.push_back(targetElem->Get<ignition::math::Pose3d>());
		targetElem = targetElem->GetNextElement("target");
	  }
	  
	  this->dataPtr->prev_target = this->dataPtr->targets[this->dataPtr->currentTarget];
   } else{
	   auto boundaryElem = _sdf->GetElement("boundary");
		if (boundaryElem){
			this->dataPtr->boundaries = boundaryElem->Get<ignition::math::Vector4d>();
	   }
	   if (_sdf->HasElement("pose")){
		    
		    this->dataPtr->prev_target = _sdf->GetElement("pose")->Get<ignition::math::Pose3d>();
		    gzdbg << "HERE\n";
	   } else{
			this->dataPtr->prev_target = ignition::math::Pose3d(0.5*(this->dataPtr->boundaries.X()+this->dataPtr->boundaries.Y()),0.5*(this->dataPtr->boundaries.Z()+this->dataPtr->boundaries.W()),1,0,0,0);
		}
		this->dataPtr->curr_target = this->dataPtr->prev_target;
   }
   
   std::printf("(%f, %f, %f, %f)\n", this->dataPtr->boundaries.X(), this->dataPtr->boundaries.Y(), this->dataPtr->boundaries.Z(), this->dataPtr->boundaries.W());

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

ignition::math::Vector3d TrajectoryActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
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
		if ((!this->dataPtr->random_walk || model->GetName() == "Myhal_5th_floor" || model->GetName() == "ground_plane") && !act && std::find(this->dataPtr->obstacles.begin(), this->dataPtr->obstacles.end(), model->GetName()) == this->dataPtr->obstacles.end())
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

			/*
			this->dataPtr->curr_vel = 0.8*(modelDist/this->dataPtr->obstacleMargin); 
			*/
			if (this->dataPtr->random_walk){
				this->dataPtr->targetPose = this->RandomTarget();
			}
			
		} 
		if (offset.Length() < (this->dataPtr->obstacleMargin)*2 && std::abs(modelActor(0, 3)) < this->dataPtr->obstacleMargin * 0.4 && modelActor(2, 3) > 0)
		{	
	
			this->dataPtr->curr_vel = 0.7; //add scaling to this
			
		}

		}
		return res;
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::UpdateTarget(const common::UpdateInfo &_info)
{
  // Current actor position
  auto actorPos = this->dataPtr->actor->WorldPose().Pos();

  // Current target
  auto target = this->dataPtr->targets[this->dataPtr->currentTarget].Pos();

  // 2D distance to target
  auto posDiff = target - actorPos;
  posDiff.Z(0);

  double distance = posDiff.Length();

  // Still far from target?
  if (distance > this->dataPtr->targetRadius)
    return;

  // Move on to next target
  this->dataPtr->lastRetarget = _info.simTime;
  this->dataPtr->prev_target = this->dataPtr->targets[this->dataPtr->currentTarget];
  this->dataPtr->currentTarget++;
  if (this->dataPtr->currentTarget > this->dataPtr->targets.size() - 1){
    this->dataPtr->currentTarget = 0;
   }
}


ignition::math::Pose3d TrajectoryActorPlugin::RandomTarget(){
	
	auto actorPos =this->dataPtr->actor->WorldPose().Pos();
	
	auto posDiff = this->dataPtr->curr_target.Pos() - actorPos;
	posDiff.Z(0);
	
	double distance = posDiff.Length();

  // Still far from target?
	if (distance > this->dataPtr->targetRadius)
		return this->dataPtr->curr_target;
	
	bool not_suitable = true;
	
	auto world = this->dataPtr->actor->GetWorld();
	double x,y;
	while (not_suitable){
		x = ignition::math::Rand::DblUniform(this->dataPtr->boundaries.X(), this->dataPtr->boundaries.Y());
		y = ignition::math::Rand::DblUniform(this->dataPtr->boundaries.Z(), this->dataPtr->boundaries.W());
		
		for (unsigned int i = 0; i < world->ModelCount(); ++i)
		{
		  ignition::math::Vector3d o_pos = world->ModelByIndex(i)->WorldPose().Pos();
		  o_pos.Z() = 0;
		
		  double dist = (o_pos -  ignition::math::Vector3d(x,y,0)).Length();
		  
		  
		  if (dist > this->dataPtr->obstacleMargin*2) 
		  {

			not_suitable = false;
		  }
		 
		}
		
	}

	
	ignition::math::Vector3d dir = ignition::math::Vector3d(x-this->dataPtr->prev_target.Pos().X(), y-this->dataPtr->prev_target.Pos().Y(), 0);
	double yaw = std::atan2(dir.Y(), dir.X());
	ignition::math::Pose3d res = ignition::math::Pose3d(x,y,1,1.5708,0,yaw);
	
	this->dataPtr->prev_target = this->dataPtr->curr_target;
	this->dataPtr->curr_target = res;
	return res;
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->dataPtr->lastUpdate).Double();

  if (dt < 1/this->dataPtr->updateFreq)
    return;

  this->dataPtr->lastUpdate = _info.simTime;

    
  // Current pose - actor is oriented Y-up and Z-front
  auto actorPose = this->dataPtr->actor->WorldPose();

  // Targeting 
  //ignition::math::Pose3d targetPose;
  if (!this->dataPtr->random_walk){
	this->UpdateTarget(_info);
	this->dataPtr->targetPose = this->dataPtr->targets[this->dataPtr->currentTarget];
  } else {
	  //select random target in bounds 
	 this->dataPtr->targetPose = this->RandomTarget();
  }
  

  // Direction to target
  auto dir = (this->dataPtr->targetPose.Pos() - actorPose.Pos()).Normalize();
  
  
  //attempting to handle obsticles
  this->dataPtr->curr_vel = this->dataPtr->velocity; 
  ignition::math::Vector3d shift = this->HandleObstacles(dir);
  actorPose.Pos() += shift * this->dataPtr->curr_vel * dt;


  // TODO: generalize for actors facing other directions
  auto currentYaw = actorPose.Rot().Yaw();

  // Difference to target
  ignition::math::Angle yawDiff = atan2(dir.Y(), dir.X()) + IGN_PI_2 - currentYaw;
  yawDiff.Normalize();

   this->dataPtr->cornerAnimation = nullptr;

    actorPose.Pos() += dir * this->dataPtr->curr_vel * dt;

    // TODO: remove hardcoded roll
    actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, currentYaw + yawDiff.Radian()*0.1);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() -
      this->dataPtr->actor->WorldPose().Pos()).Length();

  // Update actor
  this->dataPtr->actor->SetWorldPose(actorPose, false, false);
  this->dataPtr->actor->SetScriptTime(this->dataPtr->actor->ScriptTime() + (distanceTraveled * this->dataPtr->animationFactor));

}


