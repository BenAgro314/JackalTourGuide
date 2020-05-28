#include "simulator_plugin.hh"
#include <iterator>
#include <algorithm>   
#include <ctime>        
#include <cstdlib> 

GZ_REGISTER_WORLD_PLUGIN(WorldHandler)

void WorldHandler::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){

    std::srand ( unsigned ( std::time(0) ) );
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&WorldHandler::OnUpdate, this, std::placeholders::_1));

    this->world = _world;
    this->sdf = _sdf;

    // load in building links 
   
    if (this->sdf->HasElement("building")){
        std::string building_name = this->sdf->GetElement("building")->Get<std::string>();

        this->building = this->world->ModelByName(building_name);
    }
    
    this->LoadParams();

    //junk

 
    //std::shared_ptr<myhal::IncludeModel> table = std::make_shared<myhal::IncludeModel>("table", ignition::math::Pose3d(2, 0, 0, 0, 0, 0), "model://table_conference_2");
    //std::shared_ptr<myhal::IncludeModel> table2 = std::make_shared<myhal::IncludeModel>("table", ignition::math::Pose3d(-2, 0, 0, 0, 0, 0), "model://table_conference_2");
    

    //myhal::ModelGroup group = myhal::ModelGroup("t_group", ignition::math::Pose3d(0, -4, 0, 0, 0, 0), "model://table_conference_3");
    //group.AddObject("chair", ignition::math::Pose3d(0,-5,0,0,0,0), "model://chair_1");

    // std::shared_ptr<myhal::Room> main_atrium = std::make_shared<myhal::Room>(-3,-2,3,10, this->building, true);
    // main_atrium->AddModel(table);

    // this->rooms.push_back(main_atrium);
    // this->rooms[0]->AddToWorld(this->world);

    //for (auto room_info: this->rooms){
    //    this->FillRoomModels(room_info);
    //    room_info->room->AddToWorld(this->world);

        
    //}
    std::cout << "ROOMS FILLED WITH MODELS" << std::endl;

    sdf::SDF actor1;
    actor1.SetFromString(
       "<sdf version ='1.6'>\
          <actor name=\"actor1_7\">\
                <pose>0 0 1.000000 0.000000 -0.000000 0.000000</pose>\
                <skin>\
                        <filename>model://actor/meshes/SKIN_man_blue_shirt.dae</filename>\
                </skin>\
                <animation name=\"walking\">\
                        <filename>walk.dae</filename>\
                        <interpolate_x>true</interpolate_x>\
                </animation>\
                <plugin name=\"vehicle2\" filename=\"libvehicle_plugin.so\">\
                        <vehicle_type>wanderer</vehicle_type>\
                        <building>building</building>\
                        <max_speed>1</max_speed>\
                </plugin>\
            </actor>\
        </sdf>");
    this->world->InsertModelSDF(actor1);
     
    sdf::SDF actor2;
    actor2.SetFromString(
       "<sdf version ='1.6'>\
          <actor name=\"actor1_6\">\
                <pose>-2.462314 -4.018197 1.000000 0.000000 -0.000000 0.000000</pose>\
                <skin>\
                        <filename>model://actor/meshes/SKIN_man_blue_shirt.dae</filename>\
                </skin>\
                <animation name=\"walking\">\
                        <filename>walk.dae</filename>\
                        <interpolate_x>true</interpolate_x>\
                </animation>\
                <plugin name=\"vehicle\" filename=\"libvehicle_plugin.so\">\
                        <vehicle_type>wanderer</vehicle_type>\
                        <building>building</building>\
                        <max_speed>1</max_speed>\
                </plugin>\
            </actor>\
        </sdf>");
    //this->world->InsertModelSDF(actor2);
}

void WorldHandler::OnUpdate(const gazebo::common::UpdateInfo &_info){

    
    if (this->tick == 1){

        //for (auto room_info: this->rooms){
        //    this->FillRoomActors(room_info);
        //    room_info->room->AddToWorld(this->world);
       // }
        //std::cout << "ROOMS FILLED WITH ACTORS" << std::endl;
        //junk
        // std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>("vehicle", "libvehicle_plugin.so");
        // plugin->AddSubtag("vehicle_type", "wanderer");
        // plugin->AddSubtag("building", "building");
        // plugin->AddSubtag("max_speed", "1");
        // std::shared_ptr<SDFAnimation> animation = std::make_shared<SDFAnimation>("walking", "walk.dae", true);

        // std::shared_ptr<myhal::Actor> actor = std::make_shared<myhal::Actor>("guy", ignition::math::Pose3d(0, 0, 1, 0, 0, 0), "model://actor/meshes/SKIN_man_blue_shirt.dae");

        // actor->AddAnimation(animation);
        // actor->AddPlugin(plugin);

        // if(this->rooms[0]->AddModelRandomly(actor, this->world, 0.5)){
        //     this->rooms[0]->AddToWorld(this->world);
        // }

        // auto new_actor = std::make_shared<myhal::Actor>("actor", ignition::math::Pose3d(0,0,1,0,0,0), "walk.dae"); //TODO randomize initial Rot
        // auto new_actor2 = std::make_shared<myhal::Actor>("actor", ignition::math::Pose3d(0,0,1,0,0,0), "moonwalk.dae"); //TODO randomize initial Rot

        // std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>("vehicle", "libvehicle_plugin.so");
        // plugin->AddSubtag("vehicle_type", "wanderer");
        // plugin->AddSubtag("building", "building");
        // plugin->AddSubtag("max_speed", "1");
        // std::shared_ptr<SDFAnimation> animation = std::make_shared<SDFAnimation>("walking", "walk.dae", true);
       

        // new_actor->AddAnimation(animation);
        // new_actor2->AddAnimation(animation);
        // //std::cout << animation->WriteTag(0) << std::endl;
        // new_actor->AddPlugin(plugin);
        // new_actor2->AddPlugin(plugin);
        // //std::cout << plugin->WriteTag(0) << std::endl;

        
        // this->world->InsertModelSDF
        
        
        // new_actor2->AddToWorld(this->world);
        

        //this->rooms[0]->AddModel(actor);
       
        
    }
    this->tick++;
    
}

void WorldHandler::LoadParams(){

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "WorldHandler");
    ros::NodeHandle nh;


   //create list of plugins based on yaml file 
    
    std::vector<std::string> plugin_names;
    if (!nh.getParam("plugin_names", plugin_names)){
        std::cout << "ERROR READING PARAMS\n";
        return;
    }

    for (auto name: plugin_names){
        //std::cout << name << std::endl;
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }

        std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>(info["name"], info["filename"]);

        std::map<std::string, std::string>::iterator it;

        for ( it = info.begin(); it != info.end(); it++ ){
            if (it->first == "name"){
                continue;
            }
            plugin->AddSubtag(it->first, info[it->first]);
        }
        this->vehicle_plugins[info["name"]] = plugin;
       
    }

    std::vector<std::string> animation_names;
    if (!nh.getParam("animation_names", animation_names)){
        std::cout << "ERROR READING PARAMS\n";
        return;
    }

    for (auto name: animation_names){
        //std::cout << name << std::endl;
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }

        std::shared_ptr<SDFAnimation> animation = std::make_shared<SDFAnimation>(name, info["filename"], true);

        this->animation_list.push_back(animation);
       
    }

    std::vector<std::string> model_names;
    if (!nh.getParam("model_names", model_names)){
        std::cout << "ERROR READING PARAMS\n";
        return;
    }

    for (auto name: model_names){
        //std::cout << name << std::endl;
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }

        std::shared_ptr<ModelInfo> m_info = std::make_shared<ModelInfo>(name, info["filename"]);

        this->model_info[name] = m_info;
    }

    std::vector<std::string> actor_names;
    if (!nh.getParam("actor_names", actor_names)){
        std::cout << "ERROR READING PARAMS\n";
        return;
    }

    for (auto name: actor_names){
        //std::cout << name << std::endl;
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }

        std::shared_ptr<ActorInfo> a_info = std::make_shared<ActorInfo>(name, info["filename"], info["plugin"], std::stod(info["obstacle_margin"]));

        this->actor_info[name] = a_info;
    }

    std::vector<std::string> scenario_names;
    if (!nh.getParam("scenario_names", scenario_names)){
        std::cout << "ERROR READING PARAMS\n";
        return;
    }

    for (auto name: scenario_names){
        //std::cout << name << std::endl;
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }

        std::shared_ptr<Scenario> scenario = std::make_shared<Scenario>(std::stod(info["pop_density"]), std::stod(info["table_percentage"]), info["actor"]);

        std::vector<std::string> model_list; 

        if (!nh.getParam(info["model_list"], model_list)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }


        for (auto model_name: model_list){
            scenario->AddModel(this->model_info[model_name]);
        }

        this->scenarios[name] = scenario;
         std::cout << "scenario names " << name << std::endl;
    }

    std::vector<std::string> room_names;
    if (!nh.getParam("room_names", room_names)){
        std::cout << "ERROR READING PARAMS\n";
        return;
    }

    for (auto name: room_names){
        //std::cout << name << std::endl;
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }

        std::map<std::string, double> geometry; 

        if (!nh.getParam(info["geometry"], geometry)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }

        std::shared_ptr<myhal::Room> room = std::make_shared<myhal::Room>(geometry["x_min"], geometry["y_min"], geometry["x_max"], geometry["y_max"], this->building, (bool) std::stoi(info["enclosed"]));

        std::vector<double> positions;

        if (!nh.getParam(info["positions"], positions)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }

        auto r_info = std::make_shared<RoomInfo>(room, info["scenario"], positions);
        this->rooms.push_back(r_info);
        //this->FillRoomModels(room, info["scenario"], positions);
        //this->rooms[name] = room;
        
    }

}

void WorldHandler::FillRoomModels(std::shared_ptr<RoomInfo> room_info){

    // fill models in position
    std::cout << room_info->scenario << std::endl;
    auto scenario = this->scenarios[room_info->scenario];
    
    int num_models = (int) (scenario->model_percentage*((room_info->positions.size())/2));
    
    std::random_shuffle(room_info->positions.begin(), room_info->positions.end());
    
    for (int i = 0; i < num_models; ++i){
        auto m_info = scenario->GetRandomModel();
        auto random_pose = ignition::math::Pose3d(room_info->positions[i], room_info->positions[i+1], 0, 0, 0, 0); //TODO: specify randomization parameters in yaml
        if (m_info){
            auto new_model = std::make_shared<myhal::IncludeModel>(m_info->name, random_pose, m_info->filename);
            room_info->room->AddModel(new_model);
        } 
    }
    
   
}

void WorldHandler::FillRoomActors(std::shared_ptr<RoomInfo> room_info){

    // fill actors randomly 
    auto scenario = this->scenarios[room_info->scenario];
    std::cout << scenario->pop_density << std::endl;
    int num_actors = (int) ((scenario->pop_density)*(room_info->room->Area()));
    auto a_info = this->actor_info[scenario->actor];
    //auto plugin = this->vehicle_plugins[a_info->plugin];
    std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>("vehicle", "libvehicle_plugin.so");
    plugin->AddSubtag("vehicle_type", "wanderer");
    plugin->AddSubtag("building", "building");
    plugin->AddSubtag("max_speed", "1");
    
    for (int i =0; i<num_actors; i++){
        auto new_actor = std::make_shared<myhal::Actor>(a_info->name, ignition::math::Pose3d(0,0,1,0,0,0), a_info->filename); //TODO randomize initial Rot
        // for (auto animation: this->animation_list){
        //     new_actor->AddAnimation(animation);
        //     std::cout << animation->WriteTag(0) << std::endl;
        // }
        // std::cout << "animations added"<< std::endl;
        // new_actor->AddPlugin(plugin);
        // std::cout << plugin->WriteTag(0) << std::endl;
        // std::cout << "plugin added" << std::endl;

        std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>("vehicle", "libvehicle_plugin.so");
        plugin->AddSubtag("vehicle_type", "wanderer");
        plugin->AddSubtag("building", "building");
        plugin->AddSubtag("max_speed", "1");
        std::shared_ptr<SDFAnimation> animation = std::make_shared<SDFAnimation>("walking", "walk.dae", true);

        new_actor->AddAnimation(animation);
        std::cout << animation->WriteTag(0) << std::endl;
        new_actor->AddPlugin(plugin);
        std::cout << plugin->WriteTag(0) << std::endl;

        room_info->room->AddModelRandomly(new_actor, this->world, a_info->obstacle_margin);
        
    }
    
    std::cout << "done adding actors" << std::endl;
}

Scenario::Scenario(double _pop_density, double _model_percentage, std::string _actor){
    this->pop_density = _pop_density;
    this->model_percentage = _model_percentage;
    this->actor = _actor;
}

void Scenario::AddModel(std::shared_ptr<ModelInfo> model){
    this->models.push_back(model);
}
std::shared_ptr<ModelInfo> Scenario::GetRandomModel(){
    if (this->models.size() <= 0){
        std::cout << "ERROR NO MODEL FOUND" << std::endl;
        return nullptr;
    }
    int i = ignition::math::Rand::DblUniform(0, this->models.size()-1);

    return this->models[i];
}