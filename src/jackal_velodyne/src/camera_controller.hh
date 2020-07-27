#pragma once

#include "gazebo/gazebo.hh"
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <ros/ros.h>

namespace gazebo {

class CameraController: public CameraPlugin {


    private: 
            
            ros::NodeHandle nh;

    protected: 
            
            virtual void OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format);

    public:
            
            CameraController();

            void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

};


}
