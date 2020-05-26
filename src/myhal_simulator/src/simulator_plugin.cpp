#include "simulator_plugin.hh"



GZ_REGISTER_WORLD_PLUGIN(WorldHander)

void WorldHander::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){

    this->world = _world;
    this->sdf = _sdf;

    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&WorldHander::OnUpdate, this, std::placeholders::_1));

    std::cout << "HELLO WORLD" << std::endl;

    std::shared_ptr<HeaderTag> actor = std::make_unique<HeaderTag>("actor");
    actor->AddAttribute("name", "bob");
    std::shared_ptr<DataTag> data = std::make_shared<DataTag>("vehicle_type", "wanderer");
    std::shared_ptr<DataTag> pose = std::make_shared<DataTag>("pose", "1 1 0 0 0 0");
    std::shared_ptr<HeaderTag> plugin = std::make_unique<HeaderTag>("plugin");

    plugin->AddAttribute("name", "simulator");
    plugin->AddAttribute("filename","libvehicle_plugin.so");
    plugin->AddSubtag(data);

    actor->AddSubtag(plugin);
    actor->AddSubtag(pose);

    std::cout << actor->WriteTag(0) << std::endl;
}

void WorldHander::OnUpdate(const gazebo::common::UpdateInfo &_info){
    //std::cout << "HELLO WORLD" << std::endl;
}