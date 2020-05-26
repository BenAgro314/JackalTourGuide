#include "simulator_plugin.hh"



GZ_REGISTER_WORLD_PLUGIN(WorldHander)

void WorldHander::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){

    this->world = _world;
    this->sdf = _sdf;

    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&WorldHander::OnUpdate, this, std::placeholders::_1));

    /*
    std::shared_ptr<HeaderTag> actor = std::make_unique<HeaderTag>("actor");
    actor->AddAttribute("name", "bob");
    std::shared_ptr<DataTag> pose = std::make_shared<DataTag>("pose", "1 1 0 0 0 0");
    std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>("test_plugin", "libboids_plugin.so");
    std::shared_ptr<SDFAnimation> animation = std::make_shared<SDFAnimation>("walking", "walk.dae", true);

    plugin->AddSubtag("vehicle_type", "boid");

    actor->AddSubtag(plugin);
    actor->AddSubtag(pose);
    actor->AddSubtag(animation);

    std::cout << actor->WriteTag(0) << std::endl;
    */

    std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>("vehicle", "libvehicle_plugin.so");
    plugin->AddSubtag("vehicle_type", "wanderer");
    plugin->AddSubtag("building", "building");
    plugin->AddSubtag("max_speed", "1");
    std::shared_ptr<SDFAnimation> animation = std::make_shared<SDFAnimation>("walking", "walk.dae", true);

    std::shared_ptr<myhal::Actor> actor = std::make_shared<myhal::Actor>("guy", ignition::math::Pose3d(1, 1, 0, 0, 0, 0), "model://actor/meshes/SKIN_man_blue_shirt.dae");

    actor->AddAnimation(animation);
    actor->AddPlugin(plugin);

    std::cout << actor->CreateSDF() << std::endl;
}

void WorldHander::OnUpdate(const gazebo::common::UpdateInfo &_info){
    //std::cout << "HELLO WORLD" << std::endl;
}