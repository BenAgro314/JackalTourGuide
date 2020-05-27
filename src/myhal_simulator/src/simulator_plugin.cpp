#include "simulator_plugin.hh"



GZ_REGISTER_WORLD_PLUGIN(WorldHander)

void WorldHander::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){

    this->world = _world;
    this->sdf = _sdf;

    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&WorldHander::OnUpdate, this, std::placeholders::_1));

    std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>("vehicle", "libvehicle_plugin.so");
    plugin->AddSubtag("vehicle_type", "wanderer");
    plugin->AddSubtag("building", "building");
    plugin->AddSubtag("max_speed", "1");
    std::shared_ptr<SDFAnimation> animation = std::make_shared<SDFAnimation>("walking", "walk.dae", true);

    std::shared_ptr<myhal::Actor> actor = std::make_shared<myhal::Actor>("guy", ignition::math::Pose3d(0, 0, 1, 0, 0, 0), "model://actor/meshes/SKIN_man_blue_shirt.dae");

    actor->AddAnimation(animation);
    actor->AddPlugin(plugin);



    std::shared_ptr<myhal::IncludeModel> table = std::make_shared<myhal::IncludeModel>("table", ignition::math::Pose3d(2, 0, 0, 0, 0, 0), "model://table_conference_2");
    std::shared_ptr<myhal::IncludeModel> table2 = std::make_shared<myhal::IncludeModel>("table", ignition::math::Pose3d(-2, 0, 0, 0, 0, 0), "model://table_conference_2");
    
    //actor->AddToWorld(this->world);
    //table->AddToWorld(this->world);

    std::shared_ptr<myhal::Room> main_atrium = std::make_shared<myhal::Room>(-3,-6,3,0);
    
   
    main_atrium->AddModel(table);
    main_atrium->AddModel(table2);
    
    this->rooms.push_back(main_atrium);
    this->rooms[0]->AddToWorld(this->world);
    //std::cout << actor->CreateSDF() << std::endl;
    this->rooms[0]->AddModel(actor);
}

void WorldHander::OnUpdate(const gazebo::common::UpdateInfo &_info){

    
    //std::cout << "HELLO WORLD" << std::endl;
    if (this->tick == 1){
        
        this->rooms[0]->AddToWorld(this->world);
    }
    this->tick++;
    
}