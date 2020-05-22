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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//int cycles = 0;
//int max_cycles = 10;

rosbag::Bag tour;
bool start_tour = false;


void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	
	ROS_WARN("Starting Tour\n");
	start_tour = true;
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
    ros::Rate r(10);
    while (!start_tour){
      
      ros::spinOnce();  
      r.sleep();
    }

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    std::string file_name ="/home/default/catkin_ws/src/jackal_velodyne/src/include/tours/" + bag_name;
    tour.open(file_name);
    
    
    move_base_msgs::MoveBaseGoal goal;
    
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
    if ( ros::ok() ) {
      // no Crlt+C received, i. e. we have to ros::shutdown() 
      ros::shutdown();
    }
  
    return 0;
}
