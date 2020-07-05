#include "world_master.hh"

namespace gazebo{
    GZ_REGISTER_WORLD_PLUGIN(WorldMaster);
}

void WorldMaster::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){

    this->world = _world;

    this->update_connection.push_back(gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&WorldMaster::WorldUpdate, this, std::placeholders::_1)));
    this->update_connection.push_back(gazebo::event::Events::ConnectBeforePhysicsUpdate(std::bind(&WorldMaster::PhysicsUpdate, this, std::placeholders::_1)));


    auto D = dungeon::BSPDungeon(ignition::math::Box(0,0,0,10,10,2), 0.25, 0.25, 2,2 ,0.25, 1);

    D.FillCells();
    //D.ToString();
   
    D.AddToWorld(world);


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

    // sdf::SDF modelSDF;
    // modelSDF.SetFromString(
    //    "<sdf version ='1.6'>\
    //       <model name ='door'>\
    //         <pose>-5 5 1.000000 0.000000 -0.000000 1.571000</pose>\
    //         <include>\
    //             <name>door_0</name>\
    //             <uri>model://simple_door2</uri>\
	// 	    </include>\
    //       </model>\
    //     </sdf>");

    // sdf::SDF model2SDF;
    // model2SDF.SetFromString(
    //    "<sdf version ='1.6'>\
    //       <model name ='door2'>\
    //       <pose>5 5 1.000000 0.000000 -0.000000 1.571000</pose>\
    //         <include>\
    //             <name>door2</name>\
	// 	    </include>\
    //       </model>\
    //     </sdf>");

    // model2SDF.Root()->GetElement("model")->GetElement("pose")->Set(ignition::math::Pose3d(5,0,0,0,0,0));
    // model2SDF.Root()->GetElement("model")->GetElement("include")->AddElement("uri");
    // model2SDF.Root()->GetElement("model")->GetElement("include")->GetElement("uri")->Set("model://kitchen_chair");
    
    // this->world->InsertModelSDF(modelSDF);
    // this->world->InsertModelSDF(model2SDF);

   


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
    //auto B= objects::Box(ignition::math::Box(5,5,1,7,7,2));
    //B.AddToWorld(this->world);
}