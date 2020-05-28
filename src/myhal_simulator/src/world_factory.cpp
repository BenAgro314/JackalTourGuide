#include "world_factory.hh"
#include <iterator>
#include <algorithm>   
#include <ctime>        
#include <cstdlib> 
#include <iostream>
#include <fstream>

int main(int argc, char ** argv){

    auto box1 = ignition::math::Box(ignition::math::Vector3d(10,10,10),ignition::math::Vector3d(11,11,11));
    auto box2 = ignition::math::Box(ignition::math::Vector3d(-1,-1,-1),ignition::math::Vector3d(1,1,1));
    
   

    auto world_handler = WorldHandler();
    


    world_handler.Load();
    

    std::cout << "World Created" << std::endl;
}

void WorldHandler::Load(){

    //load parameters 

    this->LoadParams();

    // fill rooms
    
    for (auto r_info: this->rooms){
        this->FillRoom(r_info);
        std::cout << "before\n";
        r_info->room->AddToWorld(this->world_string);
        std::cout << "after\n";
    }

    // add all rooms to the world 

    this->WriteToFile("myhal_sim.world");
}

WorldHandler::WorldHandler(){
    this->world_string = "";
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
        this->vehicle_plugins[name] = plugin;
        //std::cout <<  name << std::endl;
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
      
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }
        //std::cout << "before " << info["width"] << " " << info["length"] << std::endl;
        std::shared_ptr<ModelInfo> m_info = std::make_shared<ModelInfo>(name, info["filename"], std::stod(info["width"]), std::stod(info["length"]));
        //std::cout << "after\n";
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
        //std::cout << info["pop_density"] << " " << info["table_percentage"] << std::endl;
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
         //std::cout << "scenario names " << name << std::endl;
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

        std::shared_ptr<myhal::Room> room = std::make_shared<myhal::Room>(geometry["x_min"], geometry["y_min"], geometry["x_max"], geometry["y_max"], (bool) std::stoi(info["enclosed"]));

        std::vector<double> poses;

        if (!nh.getParam(info["positions"], poses)){
            std::cout << "ERROR READING PARAMS\n";
            return;
        }
        std::vector<std::vector<double>> positions;
        for (int j =0; j < (int) poses.size()-1; j+=2){
            positions.push_back({poses[j],poses[j+1]});
        }
  

        auto r_info = std::make_shared<RoomInfo>(room, info["scenario"], positions);
        this->rooms.push_back(r_info);
        
        //this->rooms[name] = room;
        
    }

    //std::cout << "done\n";
}

void WorldHandler::FillRoom(std::shared_ptr<RoomInfo> room_info){

    auto scenario = this->scenarios[room_info->scenario];
    
    int num_models = (int) (scenario->model_percentage*((room_info->positions.size())));
    
    std::random_shuffle(room_info->positions.begin(), room_info->positions.end());
    
    for (int i = 0; i < num_models; ++i){
        auto m_info = scenario->GetRandomModel();
       
        auto random_pose = ignition::math::Pose3d(room_info->positions[i][0], room_info->positions[i][1], 0, 0, 0, 0); //TODO: specify randomization parameters in yaml
        if (m_info){
            auto new_model = std::make_shared<myhal::IncludeModel>(m_info->name, random_pose, m_info->filename, m_info->width, m_info->length);
            room_info->room->AddModel(new_model);
        } 
    }

    

    int num_actors = (int) ((scenario->pop_density)*(room_info->room->Area()));
    
   

    auto a_info = this->actor_info[scenario->actor];

  
    
    auto plugin = this->vehicle_plugins[a_info->plugin];

   

    for (int i =0; i<num_actors; i++){
        auto new_actor = std::make_shared<myhal::Actor>(a_info->name, ignition::math::Pose3d(0,0,1,0,0,0), a_info->filename, a_info->width, a_info->length); //TODO randomize initial Rot

        for (auto animation: this->animation_list){
            new_actor->AddAnimation(animation);
            
        }
        
        new_actor->AddPlugin(plugin);
        
        
       
        room_info->room->AddModelRandomly(new_actor);
    }

    std::cout << "Added models and actors" << std::endl;
}


void WorldHandler::WriteToFile(std::string out_name){
    std::ifstream in = std::ifstream("/home/default/catkin_ws/src/myhal_simulator/worlds/myhal_template.txt");

    if (in){
		std::cout << "Template Found\n";
	} else{
		std::cout << "Template Not Found\n";
        return;
	}

    std::ofstream out;
    out.open("/home/default/catkin_ws/src/myhal_simulator/worlds/" + out_name);

    char str[255];
	int line =0;

	while(in) {
		in.getline(str, 255);  // delim defaults to '\n'
		if(in) {
			
			if (line == 102){
				// insert writing people and furnature here
				
				out << this->world_string;
			}

			out << str << std::endl;
			line++;

		}
	}	

    out.close();
    in.close();
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
   
    int i = ignition::math::Rand::IntUniform(0, this->models.size()-1);

    return this->models[i];
}