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

    std::cout << "Running Hugues Diagnostics\n";

    std::string time_name = argv[1];

    std::string ground_truth = argv[2];

    std::string filter = argv[3];

    bool gt = false;
    if (ground_truth == "true"){
        gt = true;
    }

    

    std::cout << "Obtaining Trajectories\n";

    std::string filepath = "/home/" + user_name + "/Myhal_Simulation/simulated_runs/" + time_name + "/";

    BagTools hugues_bag = BagTools(filepath + "logs-" + time_name + "/", "hugues_test.bag");
    BagTools raw_bag = BagTools(filepath);

    auto amcl_traj = hugues_bag.GetTrajectory("/amcl_pose");
    auto gt_traj = raw_bag.GetTrajectory("/ground_truth/state");

    auto odom_to_base = raw_bag.GetTransforms("odom","base_link"); // what is the position of base_link w.r.t odom frame
    auto map_to_odom = hugues_bag.GetTransforms("hugues_map", "odom"); // what is the position of odom frame w.r.t map

    /// translate odom_traj to to map frame

    auto gmapping_traj = hugues_bag.ShiftTrajectory(odom_to_base, map_to_odom);

    std::string name = "hugues_filter";
    if (gt){
        name = "gt_filter";
    }
    if (filter == "true"){
        name = "no_filter";
    }



    if (amcl_traj.size() > 0){
        std::cout << "Computing amcl localization error\n";
        auto trans_drift = raw_bag.TranslationDrift(gt_traj, amcl_traj);
        std::cout << "Finished computing amcl localization error\n";

        std::ofstream out(filepath + "/logs-" + time_name +"/" +name +"_localization_error_amcl.csv");
        out << name << " Demon," << "Filter:," << filter << "\n";
        out << "Distance Travelled (m), AMCL Localization Error (m)\n";
        for (auto row: trans_drift){
            out << row[0] << "," << row[1] << std::endl;
        }

        out.close();
    } else if (gmapping_traj.size() >0){
       
        std::cout << "Computing gmapping localization error\n";
        auto trans_drift = raw_bag.TranslationDrift(gt_traj, gmapping_traj);
        std::cout << "Finished computing gmapping localization error\n";

        std::ofstream out(filepath + "/logs-" + time_name +"/" +name + "_localization_error_gmapping.csv");
        out << name <<" Demon," << "Filter:," <<filter << "\n";
        out << "Distance Travelled (m), Gmapping Localization Error (m)\n";
        for (auto row: trans_drift){
            out << row[0] << "," << row[1] << std::endl;
        }

        out.close();
    }

    return 0;
}