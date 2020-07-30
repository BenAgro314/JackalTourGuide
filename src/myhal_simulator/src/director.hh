#pragma once

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/Visual.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <functional>
#include <string>

#include <gazebo/common/Time.hh>
#include "utilities.hh"
#include "vehicles.hh"

namespace gazebo{

class Director: public SystemPlugin {

    private:
        
		physics::WorldPtr world = nullptr;

        std::vector<event::ConnectionPtr> connections;

        common::Time last_update = 0;

        //ros::NodeHandle nh;

        void Init();

        void Update();

        void ModelUpdates();

        rendering::CameraPtr cam;

        int count = 0;

    public:

        void Load(int, char **);

        virtual ~Director();


};

}
