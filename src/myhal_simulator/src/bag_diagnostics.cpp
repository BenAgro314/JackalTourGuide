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

    auto amcl_traj = handle.GetTrajectory("/amcl_pose");
    auto gt_traj = handle.GetTrajectory("/ground_truth/state");

    happly::PLYData plyOut;
    AddTrajectory(plyOut, amcl_traj);
    plyOut.write(filepath + "amcl_pose.ply", happly::DataFormat::Binary);

    auto trans_drift = handle.TranslationDrift(gt_traj, amcl_traj);

    std::ofstream out(filepath + "translation_drift.csv");
    out << "Time (s), Translation Drift (m)\n";
    for (auto row: trans_drift){
        out << row[0] << "," << row[1] << std::endl;
    }

    out.close();

    return 0;
}