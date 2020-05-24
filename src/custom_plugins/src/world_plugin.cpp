#include "world_plugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(Factory)

void Factory::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){

    this->world = _world;
    this->sdf = _sdf;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Factory::OnUpdate, this, std::placeholders::_1));

    //this->world->InsertModelFile("model://10x10Box");

    /*
    sdf::SDF pluginSDF;
    pluginSDF.SetFromString(
        "<sdf version ='1.6'>\
            <plugin name='trajectory' filename='libboids_plugin.so'>\
                <building>box</building>\
                <max_speed>0.866189</max_speed>\
                <obstacle_margin>0.5</obstacle_margin>\
            </plugin>\
        </sdf>"
    );
    */

    sdf::SDF actorSDF;
    actorSDF.SetFromString(
        "<sdf version ='1.6'>\
            <actor name='guy1'>\
                <skin>\
                    <filename>model://actor/meshes/SKIN_man_blue_shirt.dae</filename>\
                </skin>\
                <animation name='animation'> \
                    <filename>model://actor/meshes/ANIMATION_walking.dae</filename>\
                    <interpolate_x>true</interpolate_x>\
                </animation>\
                <plugin name='trajectory' filename='libboids_plugin.so'>\
                    <building>box</building>\
                    <max_speed>0.866189</max_speed>\
                    <obstacle_margin>0.5</obstacle_margin>\
                </plugin>\
                <plugin name='placement' filename='librandom_place_plugin.so'>\
                    <building>box</building>\
                    <obstacle_margin>0.7</obstacle_margin>\
                </plugin>\
            </actor>\
        </sdf>"
    );
    sdf::ElementPtr actor = actorSDF.Root()->GetElement("actor");
    actor->GetAttribute("name")->SetFromString("guy");

    auto actor_pos = actor->GetElement("pose");

    actor_pos->Set<ignition::math::Pose3d>(ignition::math::Pose3d(-1, -1, 1, 0, 0, 0));


    auto plugin_elem = actor->GetElement("plugin");
    plugin_elem->GetAttribute("filename")->SetFromString("librandomwalk_plugin.so");
    plugin_elem->GetElement("max_speed")->Set<double>(1);


    plugin_elem = actor->GetNextElement("plugin");

    if (plugin_elem){
   
        plugin_elem->GetAttribute("filename")->SetFromString("librandom_place_plugin.so");
    }
    this->world->InsertModelSDF(actorSDF);
    /*
    actor->GetElement("plugin")->GetAttribute("name")->SetFromString("trajectory");
    actor->GetElement("plugin")->GetAttribute("filename")->SetFromString("libboids_plugin.so");
    //actor->AddElementDescription(pluginSDF.Root()->GetElement("plugin"));
    actor->GetElement("plugin")->SetDescription(
        "<building>box</building>\
        <max_speed>0.866189</max_speed>\
        <obstacle_margin>0.5</obstacle_margin>"
    );
    */
    
   
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(
       "<sdf version ='1.6'>\
          <model name ='sphere'>\
            <pose>1 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 .5 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
    // Demonstrate using a custom model name.
    sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
    model->GetAttribute("name")->SetFromString("unique_sphere");

    auto pose_pointer = model->GetElement("pose");
    auto pose = ignition::math::Pose3d(5, 5, 0, 0, 0, 0);



    pose_pointer->Set<ignition::math::Pose3d>(pose);

    this->world->InsertModelSDF(sphereSDF);

    
}

void Factory::OnUpdate(const gazebo::common::UpdateInfo &_info){
    /*
    for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
        auto model = this->world->ModelByIndex(i);
        std::cout << model->GetName() << std::endl;
    }
    */
}