#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"
#include <utility>
#include "vehicles.hh"

using namespace gazebo;

class ModelHandler : public ModelPlugin{

	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	// Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo &_info);

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection;


    //private: Vehicle *vehicle;
    private: std::unique_ptr<Vehicle> vehicle;
	
};


