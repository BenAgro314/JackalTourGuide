#include "custom_camera.hh"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

namespace gazebo {

    GZ_REGISTER_SENSOR_PLUGIN(CustomCamera)

    CustomCamera::CustomCamera(): nh(NULL){
        this->pub = nh.advertise<sensor_msgs::Image>("test_topic", 10);
        std::cout << "LOAD_CUSTOM_CAMERA\n";
    }

    CustomCamera::~CustomCamera() { 
        ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
    }

    void CustomCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
        // Make sure the ROS node for Gazebo has already been initialized
        std::cout << "LOAD_CUSTOM_CAMERA\n";
        if (!ros::isInitialized()) {
          ROS_FATAL_STREAM_NAMED("camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
        }

        std::cout << "LOAD_CUSTOM_CAMERA\n";

        CameraPlugin::Load(_parent, _sdf);
        this->parentSensor_ = this->parentSensor;
        this->width_ = this->width;
        this->height_ = this->height;
        this->depth_ = this->depth;
        this->format_ = this->format;
        this->camera_ = this->camera;

        GazeboRosCameraUtils::Load(_parent, _sdf);
    }

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void CustomCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format) {

        common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();
        std::cout << "UPDATE\n";

        if (!this->parentSensor->IsActive()) {
            if ((*this->image_connect_count_) > 0)
            // do this first so there's chance for sensor to run once after activated
            this->parentSensor->SetActive(true);
        }
        else {
          if ((*this->image_connect_count_) > 0) {
            if (sensor_update_time < this->last_update_time_)
            {
                ROS_WARN_NAMED("camera", "Negative sensor update time difference detected.");
                this->last_update_time_ = sensor_update_time;
            }

            if (sensor_update_time - this->last_update_time_ >= this->update_period_)
            {
              this->PutCameraData(_image, sensor_update_time);
              this->PublishCameraInfo(sensor_update_time);
              this->last_update_time_ = sensor_update_time;
            }
          }
        }
    }

}
