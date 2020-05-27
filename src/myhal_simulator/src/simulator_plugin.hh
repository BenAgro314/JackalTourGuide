#ifndef SIMULATOR_PLUGIN_HH
#define SIMULATOR_PLUGIN_HH

#include <functional>
#include "world_entities.hh"
#include <utility>

class WorldHander: public gazebo::WorldPlugin{


    public: 
        
        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

        void OnUpdate(const gazebo::common::UpdateInfo &_info);

        gazebo::event::ConnectionPtr update_connection;

        gazebo::physics::WorldPtr world;

        sdf::ElementPtr sdf;

        gazebo::physics::ModelPtr building;

        std::vector<std::shared_ptr<myhal::Room>> rooms;

        //std::vector<std::shared_ptr<SDFPlugin>> vehicle_plugins;

        //std::vector<std::shared_ptr<SDFAnimation>> actor_animations;

        int tick = 0;
};

#endif