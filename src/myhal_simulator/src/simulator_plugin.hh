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

    
};

#endif