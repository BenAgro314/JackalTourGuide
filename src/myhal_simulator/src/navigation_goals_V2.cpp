#include <actionlib/client/simple_action_client.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "costmap.hh"
#include "frame.hh"
#include "parse_tour.hh"
#include "utilities.hh"
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//int cycles = 0;
//int max_cycles = 10;

rosbag::Bag tour;
bool lidar = false;
bool camera = false;

void ImageCallback(const sensor_msgs::Image::ConstPtr& msg){
	
	if (!camera){
		if (!lidar){
			ROS_WARN("CAMERA RECEIVED, waiting for lidar\n");
		} else{
			ROS_WARN("CAMERA RECEIVED\n");
		}
		
	}
	camera = true;
}

void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	
	if (!lidar){
		if (!camera){
			ROS_WARN("LIDAR RECEIVED, waiting for camera\n");
		} else{
			ROS_WARN("LIDAR RECEIVED\n");
		}
		
	}
	lidar = true;
}


int main(int argc, char ** argv){
    

    ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("tour_data", 1000);

    std::string tour_name("test1");


    if (!nh.getParam("tour_name", tour_name)){
        std::cout << "ERROR READING TOUR NAME\n";
    }

    
    TourParser parser(tour_name);
    std::vector<ignition::math::Pose3d> route = parser.GetRoute();

    std::string username = "default";
    if (const char * user = std::getenv("USER")){
        username = user;
    } 

    std::string start_time;
    if (!nh.getParam("start_time", start_time)){
        std::cout << "ERROR SETTING START TIME\n";
    }
    
    std::string filepath = "/home/" + username + "/Myhal_Simulation/simulated_runs/" + start_time + "/";
    
    std::ofstream log_file;
    log_file.open(filepath + "/logs/log.txt", std::ios_base::app);

    ROS_WARN("USING TOUR %s\n", tour_name.c_str());
	
    ros::Subscriber lidar_sub = nh.subscribe("velodyne_points", 1000, LidarCallback);
    ros::Subscriber camera_sub = nh.subscribe("kinect_V2/depth/image_raw", 1000, ImageCallback); //nh.subscribe("no_gpu_points", 1000, LidarCallback);
    ros::Rate r(10);
    while (!lidar || !camera){ // wait until both lidar and camera have been recieved 
        
        ros::spinOnce();  
        r.sleep();
    }

    for (auto pose: route){
        geometry_msgs::PoseStamped msg;
		msg.pose.position.x = pose.Pos().X();
        msg.pose.position.y = pose.Pos().Y();
        msg.pose.position.z = pose.Pos().Z();
		pub.publish(msg);
    }

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    std::string filename;
    std::string shutdown_file = "/home/"+username+"/catkin_ws/shutdown.sh";

    int count = 0;

    log_file << "\nTour diagnostics: " << std::endl;
    for (auto pose: route){
        count++;
        ROS_WARN("Sending Command (%d/%ld):", count, (long) route.size());
        ROS_WARN("Target -> (%f, %f, %f)", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        
            
            
        auto goal = PoseToGoal(pose);
        
        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Target Reached");
            
            log_file << "Reached target (" << count << "/" << route.size() << ") at position (" << std::fixed << std::setprecision(1) << pose.Pos().X() << "m, " << std::fixed << std::setprecision(1) << pose.Pos().Y()<< "m, " << std::fixed << std::setprecision(1) << pose.Pos().Z()<<  "m) at time: " << std::fixed << std::setprecision(1) << ros::Time::now().toSec() << "s" << std::endl;
        } else {
            ROS_INFO("Target Failed");
            log_file << "Failed to reach target (" << count << "/" << route.size() << ") at position (" << std::fixed << std::setprecision(1) << pose.Pos().X() << "m, " << std::fixed << std::setprecision(1) << pose.Pos().Y()<< "m, " << std::fixed << std::setprecision(1) << pose.Pos().Z()<<  "m) at time: " << std::fixed << std::setprecision(1) << ros::Time::now().toSec() << "s" << std::endl;
        }
        

    }
    
    
    
    const char *cstr = shutdown_file.c_str();
    system(cstr);
    

    
    return 0;
}


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
    }

    std::vector<double> b;
    if (!nh.getParam("bounds", b)){
        std::cout << "ERROR READING TOP LEFT CORNER";
    }

    this->bounds = ignition::math::Box(ignition::math::Vector3d(b[0], b[1],0), ignition::math::Vector3d(b[2], b[3],0));
}

void TourParser::ParseTour(){
    std::ifstream tour_file(this->tour_path + this->tour_name + ".txt");
 
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