///TODO: ADD DELAY UNTIL ALL OTHER TOPICS HAVE BEEN PUBLISHED

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <fstream>
#include <cstdlib>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//int cycles = 0;
//int max_cycles = 10;

rosbag::Bag tour;
bool lidar = false;
bool camera = false;

void ImageCallback(const sensor_msgs::Image::ConstPtr& msg){
	
	ROS_WARN("Camera Recieved!\n");
	camera = true;
}

void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	
	ROS_WARN("Lidar Recieved!\n");
	lidar = true;
}



int main(int argc, char** argv){

    
    
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle nh;

    std::string bag_name("tour_positions1.bag");
  
    if (!nh.getParam("bag_name", bag_name)){
      bag_name = "tour_positions1.bag";
    }
    
    
    
    ROS_WARN("USING TOUR %s\n", bag_name.c_str());
  
    ros::Subscriber lidar_sub = nh.subscribe("velodyne_points", 1000, LidarCallback);
    ros::Subscriber camera_sub = nh.subscribe("kinect_V2/depth/image_raw", 1000, ImageCallback);
    ros::Rate r(10);
    while (!lidar || !camera){ // wait until both lidar and camera have been recieved 
      
      ros::spinOnce();  
      r.sleep();
    }

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    std::string filename;
    std::string shutdown_file = "/home/default/catkin_ws/shutdown.sh";
    if (const char * user = std::getenv("USER")){
      std::string name(user);
      filename ="/home/" + name + "/catkin_ws/src/jackal_velodyne/src/tours/" + bag_name;
      shutdown_file = "/home/"+name+"/catkin_ws/shutdown.sh";
    } else{
      std::cout << "USER NAME NOT FOUND\n";
      filename ="/home/default/catkin_ws/src/jackal_velodyne/src/tours/" + bag_name;
    }

    
    tour.open(filename);
    
    
    move_base_msgs::MoveBaseGoal goal;
    ROS_WARN("Starting Tour\n");
    for (rosbag::MessageInstance const m: rosbag::View(tour)){
      
      ROS_WARN("\nSending Command:\n");
      geometry_msgs::PoseStamped::ConstPtr i = m.instantiate<geometry_msgs::PoseStamped>();
      ROS_WARN("Target -> (x: %f, y: %f, z: %f)\n", i->pose.position.x, i->pose.position.y, i->pose.position.z);
      if (i!= nullptr){
        
        
        goal.target_pose = *i;
        
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        
        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Target Reached");
        else
        ROS_INFO("Target Failed");
      }
    }
    
    tour.close();
    const char *cstr = shutdown_file.c_str();
    system(cstr);
    
    return 0;
}
