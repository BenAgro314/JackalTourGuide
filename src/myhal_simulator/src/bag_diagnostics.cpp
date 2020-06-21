#include "bag_tools.hh"
#include "costmap.hh"

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
    plyOut.write(filepath + "/logs/amcl_pose.ply", happly::DataFormat::Binary);

    std::cout << "Computing translation drift\n";
    auto trans_drift = handle.TranslationDrift(gt_traj, amcl_traj);
    std::cout << "Finished computing translation drift\n";

    std::ofstream out(filepath + "/logs/translation_drift.csv");
    out << "Distance (m), Translation Drift (m)\n";
    for (auto row: trans_drift){
        out << row[0] << "," << row[1] << std::endl;
    }

    out.close();

    // read the information from the static objects file to creat the costmap
    std::cout << "Creating costmap\n"; 

    happly::PLYData plyIn(filepath + "/logs/static_objects.ply");
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
    
    Costmap costmap = Costmap(boundary, 0.1);

    for (auto obj: static_objects){
        if (obj.MinZ() < 1.5 && obj.MaxZ() >10e-2){
            auto box = obj.Box();

            // TODO: inflate box
            box.Min().X()-=robot_radius;
            box.Min().Y()-=robot_radius;
            box.Max().X()+=robot_radius;
            box.Max().Y()+=robot_radius;

            costmap.AddObject(box);
        }
    }

    //std::ofstream test(filepath+"test.txt");

    //test << costmap.ToString();

    //test.close();

    // read navigation goals from tour:
  

    auto goals = handle.TourTargets();

    auto times = handle.TargetSuccessTimes();
   
    
    std::vector<std::vector<ignition::math::Vector3d>> paths;
    std::cout << "Computing optimal paths\n";

    std::vector<TrajPoint> optimal_traj;

    std::ofstream path_file(filepath + "/logs/paths.txt");
    
    for (int first = 0; first < goals.size()-1; first++){
      
        auto start = goals[first];
       
        auto end = goals[first+1];
        std::vector<ignition::math::Vector3d> path;
        
  
        if(costmap.FindPath(start.Pos(), end.Pos(), path)){
            paths.push_back(path);

            
        } else{
            break;
        }

        
        
    }

    path_file << costmap.PathString();
    path_file << std::endl <<std::endl;

    path_file.close();

    // compute length of each path 
 

    std::vector<double> optimal_lengths;
    
    for (auto path: paths){
        optimal_lengths.push_back(0);
        int count = 0;
        ignition::math::Vector3d last_pose;
        for (auto pose: path){
            optimal_traj.push_back(TrajPoint(ignition::math::Pose3d(pose, ignition::math::Quaterniond(0,0,0)),(double) count));
            if (count == 0){
                last_pose = pose;
                count++;
                continue;
            }
            count ++;
            
            optimal_lengths.back() += (pose - last_pose).Length();
            
            last_pose = pose;
        }
        
    }

    while (optimal_lengths.size() < goals.size()-1){
        optimal_lengths.push_back(-1);
    }

    // find how far the robot travelled in the times from 0->times[0], times[0]->times[1] ...

    std::vector<double> actual_lengths;

    int traj_ind = 0;
    ignition::math::Vector3d final;
    for (int i =0; i<times.size(); i++){
        double time = times[i];
        actual_lengths.push_back(0);
        int count = 0;
        ignition::math::Vector3d last_pose;
        while (traj_ind < gt_traj.size() && gt_traj[traj_ind].time <= time){
            if (count == 0){
                last_pose = gt_traj[traj_ind].pose.Pos();
                count++;
                continue;
            }
            count++;
            actual_lengths.back() += (gt_traj[traj_ind].pose.Pos() - last_pose).Length();
            last_pose = gt_traj[traj_ind].pose.Pos();
            traj_ind++;
        }
        //std::cout << actual_lengths.back() << std::endl;
        final = last_pose;
    }
    if (traj_ind < gt_traj.size()){
        actual_lengths.push_back(0);

        ignition::math::Vector3d last_pose = final;
        while(traj_ind < gt_traj.size()){
            actual_lengths.back() += (gt_traj[traj_ind].pose.Pos() - last_pose).Length();
            last_pose = gt_traj[traj_ind].pose.Pos();
            traj_ind++;
        }

    }
    
    std::cout << "Writing to file\n";

    std::ofstream out2(filepath + "/logs/path_difference.csv");

    out2 << " ,Optimal path length (m), reached goal?, actual path length (m), difference (m)\n";

    for (int i =0; i< goals.size()-1; i++){
        if (i<actual_lengths.size()){

            int status = (int) (i <times.size());
            if (optimal_lengths[i] > 0){
                out2 << "Target #" << i+1 << "," << optimal_lengths[i] << "," << status << "," << actual_lengths[i] << "," << actual_lengths[i]-optimal_lengths[i] << std::endl; 
            } else{
                out2 << "Target #" << i+1 << "," << "Unreachable" << "," << status << "," << "NA"<< "," << "NA" << std::endl; 
            }
        } else{
            if (optimal_lengths[i] > 0){
                out2 << "Target #" << i+1 << "," << optimal_lengths[i] << ",0,NA,NA\n";
            } else{
                 out2 << "Target #" << i+1 << "," << "Unreachable" << ",0,NA,NA\n";
            }
            
        }
        
    }

    out2.close();

    happly::PLYData plyOut2;
    AddTrajectory(plyOut2, optimal_traj);
    plyOut2.write(filepath + "/logs/optimal_traj.ply", happly::DataFormat::Binary);
    

    return 0;
}