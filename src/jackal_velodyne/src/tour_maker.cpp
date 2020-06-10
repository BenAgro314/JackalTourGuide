///TODO: ADD DELAY UNTIL ALL OTHER TOPICS HAVE BEEN PUBLISHED

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <fstream>
#include <cstdlib>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


rosbag::Bag tour;
int count = 0;
int num_targets;

void TargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

	count++;
	ROS_WARN("Writing Target Number %d or %d\n", count, num_targets);
    tour.write("move_base_simple/goal", ros::Time::now(), msg);
   
}



int main(int argc, char** argv){

    
    
    
    ros::init(argc, argv, "tour_maker");
    ros::NodeHandle nh;

    std::cout << "Enter new tour name (note that _tour.bag will be appended to the end):\n";
    std::string bag_name("default_tour.bag");
    std::cin >> bag_name;
    std::cout << "How many targets will the tour have:\n";
    std::cin >> num_targets;

    ROS_WARN("Writing to tour: %s\n", bag_name.c_str());

    std::string filename;
    if (const char * user = std::getenv("USER")){
      std::string name(user);
      filename ="/home/" + name + "/catkin_ws/src/jackal_velodyne/tours/" + bag_name + "_tour.bag";
    } else{
      std::cout << "USER NAME NOT FOUND\n";
      filename ="/home/default/catkin_ws/src/jackal_velodyne/tours/" + bag_name + "_tour.bag";
    }
    tour.open(filename,rosbag::bagmode::Write);
  
    ros::Subscriber target_sub = nh.subscribe("move_base_simple/goal", 1000, TargetCallback);
    ros::Rate r(10);
    while (count < num_targets){
      
      ros::spinOnce();  
      r.sleep();
    }
    
    
    
    tour.close();
  
  
    return 0;
}
