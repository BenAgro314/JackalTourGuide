#include "vehicle_plugin.hh"
#include <functional>
#include <ignition/math/Rand.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ModelHandler)

void ModelHandler::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    std::string animation = "animation";
    if (_sdf->HasElement("animation")){
    	animation = _sdf->Get<std::string>("animation");
    }

    /*
    For general vehicle we need:
    - mass
    - max force
    - max speed 
    - initial_pos
    - building name
    */

    this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelHandler::OnUpdate, this, std::placeholders::_1));

    //this->vehicle = new Wanderer::Vehicle(boost::dynamic_pointer_cast<physics::Actor>(_parent), 1, 5, 1, ignition::math::Pose3d(0,0,1,0,0,0), ignition::math::Vector3d(0,0,0), animation);
	this->vehicle = std::make_unique<Wanderer>(boost::dynamic_pointer_cast<physics::Actor>(_parent), 1, 10, 1, _parent->WorldPose(), ignition::math::Vector3d(0,0,0), animation, "box");

}

void ModelHandler::OnUpdate(const common::UpdateInfo &_info){
    this->vehicle->OnUpdate(_info);
    
}