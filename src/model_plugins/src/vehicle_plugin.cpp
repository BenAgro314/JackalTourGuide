#include "vehicle_plugin.hh"
#include <functional>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ModelHandler)

void ModelHandler::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    std::string animation = "animation";
    if (_sdf->HasElement("animation")){
    	animation = _sdf->Get<std::string>("animation");
    }

    this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelHandler::OnUpdate, this, std::placeholders::_1));

	this->vehicle = std::make_unique<Vehicle>(boost::dynamic_pointer_cast<physics::Actor>(_parent), 1, 5, 1, ignition::math::Pose3d(0,0,1,0,0,0), ignition::math::Vector3d(0,0,0), animation);

}

void ModelHandler::OnUpdate(const common::UpdateInfo &_info){
    this->vehicle->OnUpdate(_info);
    
}