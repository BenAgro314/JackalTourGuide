#include "director.hh"

namespace gazebo {


Director::~Director(){
    this->connections.clear();
}

void Director::Load(int /*_argc*/,  char ** /*argv*/){
    this->connections.push_back(
            event::Events::ConnectPreRender(std::bind(&Director::Update, this)));

    /*int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "Puppeteer");*/
}

void Director::Init() {}

void Director::Update() {
    if (this->world == nullptr){
        if (physics::has_world("default")){
            this->world = physics::get_world("default");
            std::cout << "SYSTEM PLUGIN RECIEVED WORLD" << std::endl;
        } 
        return;
    }

    rendering::ScenePtr scene = rendering::get_scene(); 

    if (!scene || !scene->Initialized()) {
        return;
    }

    std::cout << "cameras: " << scene->CameraCount() << std::endl;
    count++;
}

void Director::ModelUpdates() {
    // 60 HZ is the desired model update rate
    if ((this->world->SimTime() - last_update).Double() < (1/60.0)){
        return;
    }

    this->last_update =this->world->SimTime();

    if (!this->world->IsPaused()){
        this->world->SetPaused(true);
    }

    std::cout << "Model update" << std::endl;

    if (this->world->IsPaused()){
        this->world->SetPaused(false);
    }

}

GZ_REGISTER_SYSTEM_PLUGIN(Director)

}
