#include "custom_camera.hh"
#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CustomCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
CustomCamera::CustomCamera()
{
  std::cout << "LOAD_CUSTOM_CAMERA\n";
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
CustomCamera::~CustomCamera()
{
  ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
}

void CustomCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  std::cout << "LOAD_CUSTOM_CAMERA\n";
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  std::cout << "LOAD_CUSTOM_CAMERA\n";

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
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
    const std::string &_format)
{
  common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();
  std::cout << "UPDATE\n";

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
      if (sensor_update_time < this->last_update_time_)
      {
          ROS_WARN_NAMED("camera", "Negative sensor update time difference detected.");
          this->last_update_time_ = sensor_update_time;
      }

      // OnNewFrame is triggered at the gazebo sensor <update_rate>
      // while there is also a plugin <updateRate> that can throttle the
      // rate down further (but then why not reduce the sensor rate?
      // what is the use case?).
      // Setting the <updateRate> to zero will make this plugin
      // update at the gazebo sensor <update_rate>, update_period_ will be
      // zero and the conditional always will be true.
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
