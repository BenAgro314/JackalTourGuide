#ifndef SIMULATOR_PLUGIN_HH
#define SIMULATOR_PLUGIN_HH

#include <functional>
#include "world_entities.hh"
#include <utility>
#include <ros/ros.h>

class WorldHandler: public gazebo::WorldPlugin{

    private: 
    
        
        
        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

        void OnUpdate(const gazebo::common::UpdateInfo &_info);

        void LoadParams();

        gazebo::event::ConnectionPtr update_connection;


        gazebo::physics::WorldPtr world;

        sdf::ElementPtr sdf;

        gazebo::physics::ModelPtr building;

        std::vector<std::shared_ptr<myhal::Room>> rooms;

        

        //std::vector<std::shared_ptr<SDFAnimation>> actor_animations;

        int tick = 0;

        // filled by parameters 

        std::map<std::string, std::shared_ptr<SDFPlugin>> vehicle_plugins; //one per actor
        std::vector<std::shared_ptr<SDFAnimation>> animation_list; //added to all actors 

        
};

#endif