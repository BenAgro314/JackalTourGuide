#include "simulator_plugin.hh"



GZ_REGISTER_WORLD_PLUGIN(WorldHander)

void WorldHander::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){

    this->world = _world;
    this->sdf = _sdf;

    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&WorldHander::OnUpdate, this, std::placeholders::_1));

    std::cout << "HELLO WORLD" << std::endl;

    std::shared_ptr<DataTag> tag = std::make_shared<DataTag>("vehicle_type", "wanderer");
    std::unique_ptr<HeaderTag> super = std::make_unique<HeaderTag>("plugin");
    super->AddAttribute("name", "simulator");
    super->AddAttribute("filename","libvehicle_plugin.so");
    super->AddSubtag(tag);

    std::cout << << super->WriteTag() << std::endl;
}

void WorldHander::OnUpdate(const gazebo::common::UpdateInfo &_info){
    //std::cout << "HELLO WORLD" << std::endl;
}