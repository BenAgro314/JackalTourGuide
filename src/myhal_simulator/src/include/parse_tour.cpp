#include "parse_tour.hh"

move_base_msgs::MoveBaseGoal PoseToGoal(ignition::math::Pose3d pose){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = pose.Pos().X();
    goal.target_pose.pose.position.y = pose.Pos().Y();
    goal.target_pose.pose.position.z = pose.Pos().Z();
    goal.target_pose.pose.orientation.w = pose.Rot().W();
    goal.target_pose.pose.orientation.x = pose.Rot().X();
    goal.target_pose.pose.orientation.y = pose.Rot().Y();
    goal.target_pose.pose.orientation.z = pose.Rot().Z();
    return goal;
}

TourParser::TourParser(std::string name){
    this->username = "default";
    if (const char * user = std::getenv("USER")){
        this->username = user;
    } 

    this->tour_name = name;
    this->tour_path = "/home/" + this->username + "/catkin_ws/src/myhal_simulator/tours/" + name + "/";
    this->ReadTourParams();
    this->ParseTour();
}

void TourParser::ReadTourParams(){
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "TourParser");
    ros::NodeHandle nh;

    if (!nh.getParam("resolution", this->resolution)){
        std::cout << "ERROR READING RESOLUTION";
        this->resolution = 0.1;
    }

    std::vector<double> b;
    if (!nh.getParam("bounds", b)){
        b = {-21.55, -21.4, 21.55, 21.4};
        std::cout << "ERROR READING TOP LEFT CORNER";
    }

    this->bounds = ignition::math::Box(ignition::math::Vector3d(b[0], b[1],0), ignition::math::Vector3d(b[2], b[3],0));
}

void TourParser::ParseTour(){
    std::ifstream tour_file(this->tour_path + "map.txt");
 
    std::string line;

    
    Costmap map = Costmap(this->bounds, this->resolution);

    int row = 0;
    
    std::vector<TrajPoint> traj;

   
    while (std::getline(tour_file, line)){
        //std::cout << line.size() << std::endl;
        for (int col =0; col < line.size(); col++){
           
            if (std::isalpha(line[col])){
                int order = (int)line[col];
                ignition::math::Vector3d loc;
                map.IndiciesToPos(loc, row, col);
                ignition::math::Pose3d pose = ignition::math::Pose3d(loc, ignition::math::Quaterniond(0,0,0,1));
                traj.push_back(TrajPoint(pose, (double) order));
            }
            
        }

        row++;
    }
   
    tour_file.close();

    std::sort(traj.begin(), traj.end());

    for (auto point: traj){
        //std::cout << point.pose << " time: " << point.time << std::endl;
        this->route.push_back(point.pose);
    }

}

std::vector<ignition::math::Pose3d> TourParser::GetRoute(){
    return this->route;
}
