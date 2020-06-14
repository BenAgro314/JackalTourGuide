#include "data_processing.hh"




#define NUM_TOPICS 6

int main(int argc, char** argv){
    if (argc ==1){
        std::cout << "must input folder name and true/false\n";
        return 0;
    }

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
        user_name = user;
    } 

    std::string time_name = argv[1];
    bool classify = false;
    if (argc>2){
        std::string arg = argv[2];
        classify = (arg == "true");
    }

    std::string command = "mkdir /home/" + user_name + "/Myhal_Simulation/simulated_runs/" + time_name + "/bag_frames";
    system(command.c_str());
    

    DataProcessor processor = DataProcessor(time_name, classify);

    if (classify){
        processor.SetTopics("/ground_truth/state", {"/ground_points", "/chair_points", "/moving_actor_points", "/still_actor_points","/table_points", "/wall_points"});
    } else{
        command = "mkdir /home/" + user_name + "/Myhal_Simulation/simulated_runs/" + time_name + "/stander_pose";
        system(command.c_str());
        processor.SetTopics("/ground_truth/state", {"/velodyne_points"}, "/standing_actors");
    }
    
    processor.GetData();
    processor.WriteToPLY();
}
