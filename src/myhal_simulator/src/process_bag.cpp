# include "process_bag.hh"

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
    
    bool translate = false;
    if (argc > 2){
        translate = true;
    }

    
    
    BagProcessor processor = BagProcessor("/home/" + user_name + "/Myhal_Simulation/simulated_runs/" + time_name + "/", "/ground_truth/state", "/velodyne_points", translate);
    
    processor.GetData();
    processor.WriteToPLY();

    return 0;
}