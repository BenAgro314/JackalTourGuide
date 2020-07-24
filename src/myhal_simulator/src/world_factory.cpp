#include "world_factory.hh"
#include <iterator>
#include <algorithm>   
#include <ctime>        
#include <cstdlib> 
#include <iostream>
#include <fstream>
#include "parse_tour.hh"

int main(int argc, char ** argv){

    ROS_WARN("WORLD GENERATION BEGINNING:");

    auto world_handler = WorldHandler();

    world_handler.Load();
    

    return 0;
}

void WorldHandler::Load(){

    std::cout << "Loading Parameters\n";

    this->LoadParams();

    std::cout << "Adding cameras\n";

    this->AddCameras();

    std::cout << "Done adding cameras\n";

    std::cout << "Filling Rooms\n";
    
    for (auto r_info: this->rooms){
        this->FillRoom(r_info);
        
        r_info->room->AddToWorld(this->world_string);

    }

    std::cout << "Creating doors\n";

    for (auto door: this->doors){
        door->AddToWorld(this->world_string);
    }


    std::cout << "Writing to file\n";


    this->WriteToFile("myhal_sim.world");

    std::cout << "WORLD CREATED!\n";

}

void WorldHandler::AddCameras(){

    // Iterate through all non-empty rooms and add a camera 

    int i = 0;

    std::string path = "/tmp/";
    if (this->start_time != ""){
        path = "/home/" + this->user_name + "/Myhal_Simulation/" + "/simulated_runs/" + this->start_time + "/logs-" + this->start_time + "/videos/";
    }

    
    for (auto r_info: this->rooms){
        std::string scenario = r_info->scenario;
        if (scenario == "empty"){
            i++;
            continue;
        }
        std::string r_name = this->room_names[i];
        std::cout << "Adding camera to room " << r_name  << "\n";
        auto box = r_info->room->boundary;
        auto min_x = box.Min().X();
        auto min_y = box.Min().Y();
        auto max_x = box.Max().X();
        auto max_y = box.Max().Y();

        for (int j = 0; j < this->camera_pos.size(); j++){
            auto status = this->camera_pos[j];
            if (!status){
                continue;
            }
             
            auto x = ((j % 2) == 0) ? (min_x-2.5) : (max_x+2.5);
            auto y = (j < 2) ? (max_y + 2.5) : (min_y - 2.5);
            double yaw = 0;
            if (j == 0){
                yaw = -0.785;
            } else if (j == 1){
                yaw = -2.356;
            } else if (j == 2){
                yaw = 0.785;
            } else{
                yaw = 2.356;
            }

            auto cam = myhal::Camera(r_name + "_" + std::to_string(j), ignition::math::Pose3d(x, y, 12,0, 0.785, yaw), path);
            cam.AddToWorld(this->world_string);
        }


        i++;

    }   
    

}

WorldHandler::WorldHandler(){
    this->world_string = "";
}

void WorldHandler::LoadParams(){

    //TODO: fix error handelling 

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "WorldHandler");
    ros::NodeHandle nh;

    // READ camera info

    if (!nh.getParam("camera_pos", this->camera_pos)){
        std::cout << "ERROR READING CAMERA POS: SETTING TO ALL FALSE\n";
        this->camera_pos = {false, false, false, false};
    }

    // READ BUILDING INFO

    this->user_name = "default";
    if (const char * user = std::getenv("USER")){
        this->user_name = user;
    } 

    if (!nh.getParam("start_time", this->start_time)){
        std::cout << "ERROR READING START TIME: ANY VIDEOS WILL BE SAVED TO /tmp/\n";
        this->start_time = "";
    }

    if (!nh.getParam("tour_name", this->tour_name)){
        std::cout << "ERROR READING TOUR NAME\n";
        this->tour_name = "A_tour";
        return;
    }

    TourParser parser = TourParser(this->tour_name);


    this->route = parser.GetRoute();
    this->route.insert(this->route.begin(), ignition::math::Pose3d(ignition::math::Vector3d(0,0,0), ignition::math::Quaterniond(0,0,0,1)));
    
    this->costmap = std::make_shared<Costmap>(ignition::math::Box(ignition::math::Vector3d(-21.55,-21.4,0), ignition::math::Vector3d(21.55,21.4,0)), 0.2);

    happly::PLYData plyIn("/home/" + this->user_name + "/catkin_ws/src/myhal_simulator/params/myhal_walls.ply");
    auto static_objects = ReadObjects(plyIn);

    for (auto obj: static_objects){
        if (obj.MinZ() < 1.5 && obj.MaxZ() >10e-2){
            auto box = obj.Box();
            this->costmap->AddObject(box);
            box.Min().X()-=robot_radius;
            box.Min().Y()-=robot_radius;
            box.Max().X()+=robot_radius;
            box.Max().Y()+=robot_radius;

            this->walls.push_back(box);
        }
    }

    std::vector<ignition::math::Vector3d> paths;

    for (int i =0; i< route.size()-1; i++){
        auto start = route[i];
        auto end = route[i+1];

        std::vector<ignition::math::Vector3d> path;
        this->costmap->AStar(start.Pos(), end.Pos(), path, false);
        paths.insert(paths.end(),path.begin(),path.end());
    }

    // for (auto pt: paths){
    //      this->doors.push_back(std::make_shared<myhal::IncludeModel>("point", ignition::math::Pose3d(pt,ignition::math::Quaterniond(0,0,0,0)), "model://small_point", 0.2,0.2));
    // }

    for (auto obj: static_objects){
        if (obj.MinZ()  >= (2 - 10e-3)){
            // we are dealing with a doorway

            auto box = obj.Box();
            //std::cout << box << std::endl;

            auto pos = (box.Min() + box.Max())/2;
            pos.Z() = 0;

            int open = ignition::math::Rand::IntUniform(0,1);

            auto door = std::make_shared<myhal::IncludeModel>("door", ignition::math::Pose3d(pos, ignition::math::Quaterniond(0,0,0,0)), "model://simple_door2", 0.9, 0.15);
            
            door->pose.Pos().Z() = 1;

        
            if (box.Max().X() - box.Min().X() > 0.2){
                // horizontal door
              
                door->pose.Pos().X() -= 0.45;
            } else{
                // vertical door
                door->RotateClockwise(1.571);
                door->pose.Pos().Y() -= 0.45;
            }

            bool intersected = false;
            bool near = false;

            for (int i =0; i<paths.size()-1; i++){
                auto first = paths[i];
                first.Z() = 0;

                if ((first - pos).Length() < 1){
                    near = true;
                    auto second = paths[i+1];
                    second.Z() = 0;
                    auto line = ignition::math::Line3d(first, second);

                    auto door_box = box;
                    door_box.Min().Z() = 0;
                    door_box.Max().Z() = 0;

                    auto edges = utilities::get_box_edges(door_box);

                    for (auto edge: edges){
                        if (edge.Intersect(line) || utilities::inside_box(door_box, first, true)){
                            open = 1;
                            intersected = true;
                            break;
                        }
                    }
                }

                
            }

            if (!intersected && near){
                open = 0;
            }

            auto yaw = door->pose.Rot().Yaw();
            
            door->pose.Rot() = ignition::math::Quaterniond(0,0,yaw+(open*1.571));

            if (!intersected){
                this->doors.push_back(door); //use this for now to eliminate failed tours 
            }
            

           
        }
    }

    /// READ PLUGIN INFO
    
    std::vector<std::string> plugin_names;
    if (!nh.getParam("plugin_names", plugin_names)){
        std::cout << "ERROR READING PLUGIN NAMES\n";
        return;
    }

    for (auto name: plugin_names){
        //std::cout << name << std::endl;
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING PLUGIN PARAMS\n";
            return;
        }

        //std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>(info["name"], info["filename"]);

        std::map<std::string, std::string>::iterator it;

        for ( it = info.begin(); it != info.end(); it++ ){
            /*
            if (it->first == "name"){
                continue;
            }
            */

            auto new_plugin = std::make_shared<SDFPlugin>(it->first, it->second);
            
            /*
            if (it->first == "max_speed"){
                double speed = std::stod(info[it->first]);
                speed += ignition::math::Rand::DblUniform(-speed/5, speed/5);
                plugin->AddSubtag(it->first, std::to_string(speed));
            } else{
                plugin->AddSubtag(it->first, info[it->first]);
            }
            */

           this->vehicle_plugins[name].push_back(new_plugin);
            
        }
        //this->vehicle_plugins[name] = plugin;
    }

    /// READ ANIMATION INFO

    std::vector<std::string> animation_names;
    if (!nh.getParam("animation_names", animation_names)){
        std::cout << "ERROR READING ANIMATION NAMES\n";
        return;
    }

    for (auto name: animation_names){
        //std::cout << name << std::endl;
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING ANIMATION PARAMS\n";
            return;
        }

        std::shared_ptr<SDFAnimation> animation = std::make_shared<SDFAnimation>(name, info["filename"], true);

        this->animation_list.push_back(animation);
       
    }

    /// READ MODEL INFO

    std::vector<std::string> model_names;
    if (!nh.getParam("model_names", model_names)){
        std::cout << "ERROR READING MODEL NAMES\n";
        return;
    }

    for (auto name: model_names){
      
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING MODEL PARAMS\n";
            return;
        }
        std::shared_ptr<ModelInfo> m_info;
        if (info.find("height") != info.end()){
            m_info = std::make_shared<ModelInfo>(name, info["filename"], std::stod(info["width"])+2*this->robot_radius, std::stod(info["length"])+2*this->robot_radius, std::stod(info["height"]));
        } else{
            m_info = std::make_shared<ModelInfo>(name, info["filename"], std::stod(info["width"])+2*this->robot_radius, std::stod(info["length"])+2*this->robot_radius);
        }
        this->model_info[name] = m_info;
    }

    /// READ TABLE INFO

    std::vector<std::string> table_group_names;
    if (!nh.getParam("table_group_names", table_group_names)){
        std::cout << "ERROR READING TABLE GROUP NAMES\n";
        return;
    }

    for (auto name: table_group_names){
        std::map<std::string, std::string> info;
    

        if (!nh.getParam(name,info)){
            std::cout << "ERROR READING TABLE GROUP PARAMS\n";
            return;
        }

        std::shared_ptr<TableInfo> t_info = std::make_shared<TableInfo>(name, info["table"], info["chair"]);
        this->table_info[name] = t_info;
    }



    /// READ ACTOR INFO 

    std::vector<std::string> actor_names;
    if (!nh.getParam("actor_names", actor_names)){
        std::cout << "ERROR READING ACTOR NAMES\n";
        return;
    }

    for (auto name: actor_names){
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING ACTOR PARAMS\n";
            return;
        }

        std::shared_ptr<ActorInfo> a_info = std::make_shared<ActorInfo>(name, info["filename"], info["plugin"], std::stod(info["obstacle_margin"]));

        this->actor_info[name] = a_info;
    }

    /// READ SCENARIO INFO

    std::vector<std::string> scenario_names;
    if (!nh.getParam("scenario_names", scenario_names)){
        std::cout << "ERROR READING SCENARIO NAMES\n";
        return;
    }

    for (auto name: scenario_names){

        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING SCENARIO PARAMS\n";
            return;
        }

        std::shared_ptr<Scenario> scenario = std::make_shared<Scenario>(std::stod(info["pop_density"]), std::stod(info["table_percentage"]), info["actor"]);

        std::vector<std::string> model_list; 

        if (!nh.getParam(info["model_list"], model_list)){
            std::cout << "ERROR READING MODEL LIST\n";
            return;
        }


        for (auto model_name: model_list){
            scenario->AddModel(this->model_info[model_name]);
        }


        std::vector<std::string> table_group_list; 

        if (!nh.getParam(info["table_group_list"], table_group_list)){
            std::cout << "ERROR READING TABLE GROUP LIST\n";
            return;
        }


        for (auto table_group_name: table_group_list){
            scenario->AddTable(this->table_info[table_group_name]);
        }

        this->scenarios[name] = scenario;
    }


    /// READ ROOM INFO

    if (!nh.getParam("room_names", this->room_names)){
        std::cout << "ERROR READING ROOM NAMES\n";
        return;
    }

    for (auto name: this->room_names){
        //std::cout << name << std::endl;
        
        std::map<std::string, std::string> info;
        if (!nh.getParam(name, info)){
            std::cout << "ERROR READING ROOM PARAMS\n";
            return;
        }

        std::map<std::string, double> geometry; 

        if (!nh.getParam(info["geometry"], geometry)){
            std::cout << "ERROR READING ROOM GEOMETRY\n";
            return;
        }

        std::shared_ptr<myhal::Room> room = std::make_shared<myhal::Room>(geometry["x_min"], geometry["y_min"], geometry["x_max"], geometry["y_max"], this->walls, this->route, (bool) std::stoi(info["enclosed"]));

        std::vector<double> poses;

        if (!nh.getParam(info["positions"], poses)){
            std::cout << "ERROR READING POSITION PARAMS\n";
            return;
        }
        std::vector<std::vector<double>> positions;
        for (int j =0; j < (int) poses.size()-1; j+=2){
            positions.push_back({poses[j],poses[j+1]});
        }
  
       
        auto r_info = std::make_shared<RoomInfo>(room, info["scenario"], positions);
        this->rooms.push_back(r_info);
        
    }

}

void WorldHandler::FillRoom(std::shared_ptr<RoomInfo> room_info){

    if ( this->scenarios.find(room_info->scenario) == this->scenarios.end()){
        std::cout << "ERROR: YOU HAVE SPECIFIED A SCENARIO THAT DOESN'T EXIST" << std::endl;
        return;
    }
    auto scenario = this->scenarios[room_info->scenario];
    
    int num_models = (int) (scenario->model_percentage*((room_info->positions.size())));
    
    std::random_shuffle(room_info->positions.begin(), room_info->positions.end());
    
    for (int i = 0; i < num_models; ++i){

        auto t_info = scenario->GetRandomTable();

        auto random_pose = ignition::math::Pose3d(room_info->positions[i][0], room_info->positions[i][1], 0, 0, 0, 0); //TODO: specify randomization parameters in yaml
        if (t_info){
            //auto new_model = std::make_shared<myhal::IncludeModel>(m_info->name, random_pose, m_info->filename, m_info->width, m_info->length);
            if (!this->model_info.count(t_info->table_name)){
                std::cout << "TABLE NAME ERROR\n";
                return;
            } 

            if (!this->model_info.count(t_info->chair_name)){
                std::cout << "CHAIR NAME ERROR\n";
                return;
            } 

            
            auto t_model_info = this->model_info[t_info->table_name];

            auto c_model_info = this->model_info[t_info->chair_name];
            
            auto table_model = std::make_shared<myhal::IncludeModel>(t_model_info->name, random_pose, t_model_info->filename, t_model_info->width, t_model_info->length);
            auto chair_model = std::make_shared<myhal::IncludeModel>(c_model_info->name, random_pose, c_model_info->filename, c_model_info->width, c_model_info->length);

            double rotation = 1.5707 * ignition::math::Rand::IntUniform(0,1);
            auto table_group = std::make_shared<myhal::TableGroup>(table_model, chair_model, ignition::math::Rand::IntUniform(0,4), rotation); 
            room_info->room->AddModel(table_group->table_model);
            
            for (auto chair: table_group->chairs){
                room_info->room->AddModel(chair);
                // 50% chance of someone sitting on the chair 
                if (ignition::math::Rand::IntUniform(0,1) == 1){
                   
                    //std::shared_ptr<SDFPlugin> plugin = std::make_shared<SDFPlugin>("sitter_plugin", "libvehicle_plugin.so");
                    auto sitter_plugin = std::make_shared<SDFPlugin>("vehicle_type", "sitter");
                    auto chair_plugin = std::make_shared<SDFPlugin>("chair", chair->name);
                    //plugin->AddSubtag("vehicle_type", "sitter");
                    //plugin->AddSubtag("chair", chair->name);
                    auto sit_pose = chair->pose;
                    sit_pose.Pos().Z() = c_model_info->height;
                    auto sitter = std::make_shared<myhal::Actor>("sitter", sit_pose, "sitting.dae", 0.5, 0.5);
                    for (auto animation: this->animation_list){
                        sitter->AddAnimation(animation);
                    }
                    //sitter->AddPlugin(plugin);
                    sitter->AddPlugin(sitter_plugin);
                    sitter->AddPlugin(chair_plugin);
                    room_info->room->models.push_back(sitter); 
                }

            }
        

            
        } 

         
    }


    int num_actors = (int) ((scenario->pop_density)*(room_info->room->Area()));
    auto a_info = this->actor_info[scenario->actor];
    //auto plugin = this->vehicle_plugins[a_info->plugin];

    for (int i =0; i<num_actors; i++){
        auto new_actor = std::make_shared<myhal::Actor>(a_info->name, ignition::math::Pose3d(0,0,1,0,0,ignition::math::Rand::DblUniform(0,6.28)), a_info->filename, a_info->width, a_info->length); //TODO randomize initial Rot

        for (auto animation: this->animation_list){
            new_actor->AddAnimation(animation);
            
        }
        auto plugin_list = this->vehicle_plugins[a_info->plugin];
       
        for (auto plugin: plugin_list){
            new_actor->AddPlugin(plugin);
        }
        
        
        //new_actor->AddPlugin(plugin);
        
        
       
        room_info->room->AddModelRandomly(new_actor);
    }

}


void WorldHandler::WriteToFile(std::string out_name){

    

    std::string in_string = "/home/" + this->user_name + "/catkin_ws/src/myhal_simulator/worlds/myhal_template.txt";
    std::string out_string = "/home/" + this->user_name + "/catkin_ws/src/myhal_simulator/worlds/" + out_name;

    std::ifstream in = std::ifstream(in_string);

    if (in){
        ROS_INFO("TEMPLATE FILE FOUND");
	} else{
        ROS_ERROR("TEMPLATE FILE NOT FOUND");
        return;
	}

    std::ofstream out;
    out.open(out_string);

    char str[255];
	int line =0;

	while(in) {
		in.getline(str, 255);  // delim defaults to '\n'
		if(in) {
			
			if (line == 112){
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

void Scenario::AddTable(std::shared_ptr<TableInfo> table){
    this->tables.push_back(table);
}

std::shared_ptr<ModelInfo> Scenario::GetRandomModel(){
    if (this->models.size() <= 0){
        std::cout << "ERROR NO MODEL FOUND" << std::endl;
        return nullptr;
    }
   
    int i = ignition::math::Rand::IntUniform(0, this->models.size()-1);

    return this->models[i];
}

std::shared_ptr<TableInfo> Scenario::GetRandomTable(){
    if (this->tables.size() <= 0){
        std::cout << "ERROR NO TABLE FOUND" << std::endl;
        return nullptr;
    }
   
    int i = ignition::math::Rand::IntUniform(0, this->tables.size()-1);

    return this->tables[i];
}
