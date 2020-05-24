#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"

using namespace gazebo;

class ModelHandler : public ModelPlugin{

	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	// Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo &_info);

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

	
};



class Vehical{

    protected: 

        physics::ModelPtr model;
        double mass;
        double max_force;
        double max_speed;
        ignition::math::Pose3d pose;
        ignition::math::Vector3d velocity;
        

    public:

        Vehical(physics::ModelPtr _model, 
        double _mass, 
        double _max_force,
        double _max_speed,
        ignition::math::Pose3d initial_pose, 
        ignition::math::Vector3d initial_velocity = ignition::math::Vector3d(0,0,0)){

            this->model = _model;
            this->mass = _mass;
            this->max_force = _max_force;
            this->max_speed = _max_speed;
            this->pose = initial_pose;
            this->velocity = initial_velocity;

        }


};