#include "vehical_plugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ModelHandler)

void ModelHandler::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelHandler::OnUpdate, this, std::placeholders::_1));

	
}

void ModelHandler::OnUpdate(const common::UpdateInfo &_info){
    
    
}