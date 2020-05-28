#include "simulator_plugin.hh"
#include <iterator>

GZ_REGISTER_WORLD_PLUGIN(WorldHandler)

void WorldHandler::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf){

    this->world = _world;
    this->sdf = _sdf;

    // load in building links 
   
    if (this->sdf->HasElement("building")){
        std::string building_name = this->sdf->GetElement("building")->Get<std::string>();

        this->building = this->world->ModelByName(building_name);
    }
    
    this->LoadParams();

    // set variables 

   

    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&WorldHandler::OnUpdate, this, std::placeholders::_1));

    
  

    //junk

 
    std::shared_ptr<myhal::IncludeModel> table = std::make_shared<myhal::IncludeModel>("table", ignition::math::Pose3d(2, 0, 0, 0, 0, 0), "model://table_conference_2");
    //std::shared_ptr<myhal::IncludeModel> table2 = std::make_shared<myhal::IncludeModel>("table", ignition::math::Pose3d(-2, 0, 0, 0, 0, 0), "model://table_conference_2");
    

    //myhal::ModelGroup group = myhal::ModelGroup("t_group", ignition::math::Pose3d(0, -4, 0, 0, 0, 0), "model://table_conference_3");
    //group.AddObject("chair", ignition::math::Pose3d(0,-5,0,0,0,0), "model://chair_1");

    // std::shared_ptr<myhal::Room> main_atrium = std::make_shared<myhal::Room>(-3,-2,3,10, this->building, true);
    // main_atrium->AddModel(table);

    // this->rooms.push_back(main_atrium);
    // this->rooms[0]->AddToWorld(this->world);
    
    
}

void WorldHandler::OnUpdate(const gazebo::common::UpdateInfo &_info){

    
    if (this->tick == 1){

        
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

        std::shared_ptr<Scenario> scenario = std::make_shared<Scenario>(std::stod(info["pop_density"]), std::stod(info["table_percentage"]), info["vehicle_plugin"]);

        std::vector<std::string> model_list; 

        if (!nh.getParam(info["model_list"], model_list)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }


        for (auto model_name: model_list){
            scenario->AddModel(this->model_info[model_name]);
        }

        this->scenarios[name] = scenario;
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

        this->FillRoom(room, info["scenario"], positions);

        this->rooms[name] = room;
    }

}

void WorldHandler::FillRoom(std::shared_ptr<myhal::Room> room, std::string scenario, std::vector<double> positions){
    return;
}

Scenario::Scenario(double _pop_denisty, double _model_percentage, std::string _vehicle_plugin){
    this->pop_denisty = _pop_denisty;
    this->model_percentage = _model_percentage;
    this->vehicle_plugin = _vehicle_plugin;
}

void Scenario::AddModel(std::shared_ptr<ModelInfo> model){
    this->models.push_back(model);
}