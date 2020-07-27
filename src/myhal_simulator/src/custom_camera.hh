#pragma once

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>

namespace gazebo {

    class CustomCamera : public CameraPlugin, GazeboRosCameraUtils {

        public: 
        
            CustomCamera();

            ~CustomCamera();

            void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    protected: 
            
            virtual void OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    private: 
            
            ros::NodeHandle nh;

            ros::Publisher pub;
    

    };

}

