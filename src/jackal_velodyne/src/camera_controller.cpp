#include "camera_controller.hh"

namespace gazebo {

CameraController::CameraController(): CameraPlugin() {}

void CameraController::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {

    CameraPlugin::Load(_parent, _sdf);

    //std::cout << "Loading Custom Camera: " << this->parentSensor->Name() << std::endl;
    print_color("Loading Custom Camera: " + this->parentSensor->Name(), EMPHBLUE);

    if (_sdf->HasElement("filepath")){
        this->filepath = _sdf->GetElement("filepath")->Get<std::string>();
    } else {
        this->filepath = "/tmp/";
    }

    int argc = 0;

    char **argv = NULL;
    
    ros::init(argc, argv, this->parentSensor->Name());
    
    this->pause_gazebo = this->nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    this->play_gazebo = this->nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    
    this->last_update = ros::Time::now();

    if (gazebo::physics::has_world("default")){
        this->world = gazebo::physics::get_world("default");
        print_color(this->parentSensor->Name() + " found world!", EMPHBLUE);
    } else {
        print_color(this->parentSensor->Name() + " failed to find world, aborting", EMPHRED);    
        return;
    }

    std::string command = "mkdir " + this->filepath;
    std::system(command.c_str());

}

void CameraController::OnNewFrame(const unsigned char *_image, 
        unsigned int _width, 
        unsigned int _height, 
        unsigned int _depth, 
        const std::string &_format) {

    if (!this->world->IsPaused()){
        this->world->SetPaused(true);
    }


    char name[1024];
    snprintf(name, sizeof(name), "%s-%05d.jpg",
      this->parentSensor->Name().c_str(), this->save_count);

    std::string filename = this->filepath + std::string(name);


    this->parentSensor->Camera()->SaveFrame(
        _image, _width, _height, _depth, _format, filename);

    this->save_count++;

    if (this->world->IsPaused()){
        this->world->SetPaused(false);
    }

}


GZ_REGISTER_SENSOR_PLUGIN(CameraController)

}


 
