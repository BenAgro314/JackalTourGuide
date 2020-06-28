#include "bag_tools.hh"
#include "costmap.hh"

int main(int argc, char ** argv){

    if (argc <=2){
        std::cout << "must input folder name and true/false\n";
        return 0;
    }

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
        user_name = user;
    } 

    std::string time_name = argv[1];

    std::string filter_status = argv[2];

    //std::cout << "FILTER:" << filter_status << std::endl;

    std::string filepath = "/home/" + user_name + "/Myhal_Simulation/simulated_runs/" + time_name + "/";

    BagTools handle = BagTools(filepath);

    auto amcl_traj = handle.GetTrajectory("/amcl_pose");
    auto gt_traj = handle.GetTrajectory("/ground_truth/state");

    

    /// read in transforms between odom->base and map->odom

    auto odom_to_base = handle.GetTransforms("odom","base_link"); // what is the position of base_link w.r.t odom frame
    auto map_to_odom = handle.GetTransforms("map", "odom"); // what is the position of odom frame w.r.t map

    /// translate odom_traj to to map frame

    auto gmapping_traj = handle.ShiftTrajectory(odom_to_base, map_to_odom);

    std::vector<TrajPoint> estimated_traj;

    if (amcl_traj.size() > 0){
        happly::PLYData plyOut;
        AddTrajectory(plyOut, amcl_traj);
        plyOut.write(filepath + "/logs-" + time_name + "/amcl_pose.ply", happly::DataFormat::Binary);
        

        std::cout << "Computing amcl localization error\n";
        auto trans_drift = handle.TranslationDrift(gt_traj, amcl_traj);
        std::cout << "Finished computing amcl localization error\n";

        std::ofstream out(filepath + "/logs-" + time_name +"/localization_error.csv");
        if (filter_status == "true"){
            out << "Ground Truth Demon\n";
        } else {
            out << "No Demon\n";
        }
        out << "Distance Travelled (m), AMCL Localization Error (m)\n";
        for (auto row: trans_drift){
            out << row[0] << "," << row[1] << std::endl;
        }

        out.close();
    } else if (gmapping_traj.size() >0){
        happly::PLYData plyOut;
        AddTrajectory(plyOut, gmapping_traj);
        plyOut.write(filepath + "/logs-" + time_name + "/gmapping_pose.ply", happly::DataFormat::Binary);

        std::cout << "Computing gmapping localization error\n";
        auto trans_drift = handle.TranslationDrift(gt_traj, gmapping_traj);
        std::cout << "Finished computing gmapping localization error\n";

        std::ofstream out(filepath + "/logs-" + time_name +"/localization_error.csv");
        if (filter_status == "true"){
            out << "Ground Truth Demon\n";
        } else {
            out << "No Demon\n";
        }
        
        out << "Distance Travelled (m), Gmapping Localization Error (m)\n";
        for (auto row: trans_drift){
            out << row[0] << "," << row[1] << std::endl;
        }

        out.close();
    }

    // read the information from the static objects file to creat the costmap
    std::cout << "Creating costmap\n"; 

    happly::PLYData plyIn(filepath + "/logs-" + time_name +"/static_objects.ply");
    auto static_objects = ReadObjects(plyIn);

    double min_x = 10e9;
    double min_y = 10e9;
    double max_x = -10e9;
    double max_y = -10e9;

    for (auto obj: static_objects){
        min_x = std::min(obj.MinX(), min_x);
        min_y = std::min(obj.MinY(), min_y);
        max_x = std::max(obj.MaxX(), max_x);
        max_y = std::max(obj.MaxY(), max_y);
    }

    auto boundary = ignition::math::Box(min_x-1, min_y-1, 0, max_x+1, max_y+1, 0);

    double robot_radius = std::sqrt((0.21*0.21) + (0.165*0.165));
    double reso = 0.1;
    Costmap costmap = Costmap(boundary, reso);

    for (auto obj: static_objects){
        if (obj.MinZ() < 1.5 && obj.MaxZ() >10e-2){
            auto box = obj.Box();

            box.Min().X()-=robot_radius;
            box.Min().Y()-=robot_radius;
            box.Max().X()+=robot_radius;
            box.Max().Y()+=robot_radius;

            costmap.AddObject(box);
        }
    }

    auto waypoints = handle.TourTargets(); // stores all the waypoints on the tour (including the start 0,0,0)
    auto times = handle.TargetTimes(); // stores the times (and a boolean) for each target the robot tried to reach


    std::vector<double> actual_lengths;
    std::vector<double> optimal_lengths;

    std::vector<std::vector<ignition::math::Vector3d>> paths;
    std::cout << "Computing optimal paths\n";

    

    
    
    for (int first = 0; first < waypoints.size()-1; first++){
      
        auto start = waypoints[first];
        auto end = waypoints[first+1];

        std::vector<ignition::math::Vector3d> path;
  
        if(costmap.ThetaStar(start.Pos(), end.Pos(), path)){
            paths.push_back(path); 
           
            optimal_lengths.push_back(handle.PathLength(path)); 
            //std::cout << "Optimal Length To Reach Target #" <<first +1 << " is " << optimal_lengths.back() << "m" << std::endl;
            //std::cout << start.Pos() << " " << end.Pos() << std::endl;
        } else{
            optimal_lengths.push_back(-1);
            //std::cout << "Unreachable target #" << first+1 << std::endl;
        }   
    }

    int traj_ind = 0;

    for (int i =0; i<times.size(); ++i){
        double time = times[i].time;
        std::vector<ignition::math::Vector3d> temp_path;
        while (traj_ind < gt_traj.size() && gt_traj[traj_ind].time <=time){
            temp_path.push_back(gt_traj[traj_ind].pose.Pos());
            traj_ind++;
        }
        actual_lengths.push_back(handle.PathLength(temp_path));
    }

    std::vector<ignition::math::Vector3d> temp_path;
    while(traj_ind < gt_traj.size()){
        temp_path.push_back(gt_traj[traj_ind].pose.Pos());
        traj_ind ++;
    }
    actual_lengths.push_back(handle.PathLength(temp_path));

    int path_count = handle.MessageCount("/move_base/NavfnROS/plan");

    
    
    
    std::cout << "Writing to file\n";

    std::ofstream out2(filepath + "/logs-" + time_name + "/path_data.csv");

    if (filter_status == "true"){
        out2 << "Ground Truth Demon,";
    } else {
        out2 << "No Demon,";
    }
    out2 << "Numer of path computations:," << path_count << "\n";
    out2 << " ,Optimal path length (m), reached goal?, actual path length (m)\n";

    for (int i =0; i< waypoints.size()-1; ++i){
        out2 << "Target #" << i+1 << ",";
        
        double opt = optimal_lengths[i];

        opt = std::max(0.0, optimal_lengths[i]-0.5);
        

        out2 << opt << ",";
        int goal_status = 0;
        if (i < times.size()){
            goal_status = (int) times[i].success;
        } 
        out2 << goal_status << ",";

        if (i < actual_lengths.size()){
            out2 << actual_lengths[i] << "\n";
        } else {
            out2 << "NA\n";
        }
       

    }

    out2.close();

    happly::PLYData plyOut2;
    std::ofstream path_file(filepath + "/logs-" + time_name + "/paths.txt");

    int curr_ind = 0;
    
    std::vector<TrajPoint> plot_path;

    std::vector<ignition::math::Vector3d> optimal_traj;
    for (auto path: paths){
        for (auto pt: path){
            optimal_traj.push_back(pt);
        }
    }
    
    
    while (curr_ind+1 < optimal_traj.size()){
        plot_path.push_back(TrajPoint(ignition::math::Pose3d(optimal_traj[curr_ind],ignition::math::Quaterniond(0,0,0,0)), (double)1));

        auto dir = optimal_traj[curr_ind+1] - optimal_traj[curr_ind];
        double len = dir.Length();
        if (len < reso){
            curr_ind ++;
            continue;
        }
        dir = dir.Normalize();
        dir*=reso;

        auto add = dir;
        while (add.Length() < len){
            auto pt = optimal_traj[curr_ind]+add;
            plot_path.push_back(TrajPoint(ignition::math::Pose3d(pt, ignition::math::Quaterniond(0,0,0,0)), (double)1));
            add += dir;
        }
        //std::cout << curr_ind << std::endl;
        curr_ind++;
    }

    path_file << costmap.PathString(plot_path);
    path_file << std::endl <<std::endl;

    path_file.close();

    AddTrajectory(plyOut2, plot_path);
    plyOut2.write(filepath + "/logs-" + time_name + "/optimal_traj.ply", happly::DataFormat::Binary);


    return 0;
}