#include "camera_controller.hh"

namespace gazebo {

CameraController::CameraController(): CameraPlugin() {}

void CameraController::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    std::cout << "Loading Custom Camera\n";
    CameraPlugin::Load(_parent, _sdf);
}

void CameraController::OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format) {
    std::cout << "recieved message\n";
}


GZ_REGISTER_SENSOR_PLUGIN(CameraController)

}


/*  class CameraDump : public CameraPlugin
  {
    public: CameraDump() : CameraPlugin(), saveCount(0) {}

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      // Don't forget to load the camera plugin
      CameraPlugin::Load(_parent, _sdf);
    }

    // Update the controller
    public: void OnNewFrame(const unsigned char *_image,
        unsigned int _width, unsigned int _height, unsigned int _depth,
        const std::string &_format)
    {
      char tmp[1024];
      snprintf(tmp, sizeof(tmp), "/tmp/%s-%04d.jpg",
          this->parentSensor->Camera()->Name().c_str(), this->saveCount);

      if (this->saveCount < 10)
      {
        this->parentSensor->Camera()->SaveFrame(
            _image, _width, _height, _depth, _format, tmp);
        gzmsg << "Saving frame [" << this->saveCount
              << "] as [" << tmp << "]\n";
        this->saveCount++;
      }
    }

    private: int saveCount;
  };*/

 
