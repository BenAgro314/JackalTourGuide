#pragma once 

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include "objects.hh"
#include "maze.hh"
#include <map>


class WorldMaster: public gazebo::WorldPlugin{

    private:

        gazebo::physics::WorldPtr world;

        std::vector<gazebo::event::ConnectionPtr> update_connection;

        double update_freq = 100;

        gazebo::common::Time last_world_update;

        gazebo::common::Time last_physics_update;

        bool first_update = true;

        std::map<std::string, boost::shared_ptr<objects::Object>> objects;

    public: 
        
        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

        void WorldUpdate(const gazebo::common::UpdateInfo &_info);

        void PhysicsUpdate(const gazebo::common::UpdateInfo &_info);

        void ReadSDF();

        void FirstUpdate();
};