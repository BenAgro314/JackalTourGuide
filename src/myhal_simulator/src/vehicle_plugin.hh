#ifndef VEHICLE_PLUGIN_HH
#define VEHICLE_PLUGIN_HH

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

    public: void ReadSDF(sdf::ElementPtr _sdf);

    public: void ReadParams();

    public: void ReadPath(sdf::ElementPtr _sdf);

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection;


    private: std::unique_ptr<Vehicle> vehicle;

    private: std::string vehicle_type;
    private: std::string building;
    private: double max_speed;

    //for all vehicles 
    private: std::vector<double> vehicle_params;

    //for boids
    private: std::vector<double> boid_params;

    //for path follower
    private: std::shared_ptr<utilities::Path> path;

    //for stander
    private: double walking_duration = 0;
    private: double standing_duration = 5;

    //for follower

    private: std::string leader;

    //for sitter

    private: std::string chair;
	
};

#endif