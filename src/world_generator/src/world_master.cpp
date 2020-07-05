#include "world_master.hh"

namespace gazebo{
    GZ_REGISTER_WORLD_PLUGIN(WorldMaster);
}

void WorldMaster::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){

    this->world = _world;

    this->update_connection.push_back(gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&WorldMaster::WorldUpdate, this, std::placeholders::_1)));
    this->update_connection.push_back(gazebo::event::Events::ConnectBeforePhysicsUpdate(std::bind(&WorldMaster::PhysicsUpdate, this, std::placeholders::_1)));


    auto D = dungeon::BSPDungeon(ignition::math::Box(-15,4,0,15,15,2), 0.5, 0.5, 5,5 ,0.5, 1);
    objects[D.boxes->Name()] = D.boxes;
    D.FillCells();
    D.AddToWorld(world);

    
    for (double x =0; x<10; x+=3){
        auto model = boost::make_shared<objects::Model>("model",ignition::math::Pose3d(x,0,0,0,0,0), "model://table");
        objects[model->Name()] = model;
        model->AddToWorld(world);
    }

    // sdf::SDF actorSDF;
    // actorSDF.SetFromString(
    //     "<sdf version ='1.6'>\
    //         <actor name='actor1'>\
    //             <pose>0 0 1.000000 0.000000 -0.000000 0</pose>\
    //             <skin>\
    //                 <filename>model://actor/meshes/SKIN_man_blue_shirt.dae</filename>\
    //             </skin>\
    //             <animation name='walking'>\
    //                 <filename>model://actor/meshes/ANIMATION_walking.dae</filename>\
    //                 <interpolate_x>true</interpolate_x>\
    //             </animation>\
    //         </actor>\
    //     </sdf>"
    // );

    // auto actor = actorSDF.Root()->GetElement("actor");
    // actor->GetAttribute("name")->SetFromString("actor");

   


}

void WorldMaster::WorldUpdate(const gazebo::common::UpdateInfo &_info){

    if (1/(_info.simTime - this->last_world_update).Double() > this->update_freq){
        return;
    }

    if (first_update){
        this->FirstUpdate();
        first_update = false;
    }



    this->last_world_update = _info.simTime;
}

void WorldMaster::PhysicsUpdate(const gazebo::common::UpdateInfo &_info){
    if (1/(_info.simTime - this->last_physics_update).Double() > this->update_freq){
        return;
    }

    this->last_physics_update = _info.simTime;
}

void WorldMaster::ReadSDF(){

}

void WorldMaster::FirstUpdate(){
  

    for (int i =0; i<world->ModelCount(); i++){
     
        auto model = world->ModelByIndex(i);

        if (objects[model->GetName()] == nullptr){
            continue;
        }
        // if (model->GetName() == "ground_plane"){
        //     continue;
        //}
        
        objects[model->GetName()]->Model() = model;
    }
}