#include "bag_tools.hh"

int main(int argc, char ** argv){

    if (argc ==1){
        std::cout << "must input folder name and true/false\n";
        return 0;
    }

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
        user_name = user;
    } 

    std::string time_name = argv[1];

    

    std::string filepath = "/home/" + user_name + "/Myhal_Simulation/simulated_runs/" + time_name + "/";

    BagTools handle = BagTools(filepath);

    /*
    1. Read topics from bag and rewrite to new bag file 
    2. Read lidar frames from bag file 
    3. Read frames from hugues files 
            for each frame in hughes files:
                match this frame with the frame in the bag file 
                for each point in the matched frame:
                    change it's intensity value to hughes classifications
                rewrite the frame to the bag file 
    */


    handle.NewLidarTopic("/home/" + user_name + "/Myhal_Simulation/annotated_frames/" + time_name + "/");

    return 0;
}

