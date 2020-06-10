#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <vector>

int main(int argc, char** argv){
    if (argc == 1){
        std::cout << "must input bag name\n";
        return 0;
    }

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
      user_name = user;
    } 

    std::string bag_name = argv[1];
    std::string filename = bag_name;
    if (bag_name.size() > 5){
        
        filename = bag_name.substr(1,bag_name.size()-5);
    }
    rosbag::Bag bag;
    
    bag.open("/home/" + user_name + "/Myhal_Simulation/raw_bag_files/" + bag_name, rosbag::bagmode::Read);
    std::string command = "mkdir /home/" + user_name + "/Myhal_Simulation/simulated_runs/" + filename;
    system(command.c_str());
    std::ofstream gt_file;
    gt_file.open("/home/" + user_name + "/Myhal_Simulation/simulated_runs/" +  filename + "/gt_pose.csv");

    // extract topic: /ground_truth/state type: nav_msgs/Odometry    
    std::vector<std::string> topics = {"/ground_truth/state"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    gt_file << "id, time, pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w\n";

    for (auto msg: view){
        auto pose = msg.instantiate<nav_msgs::Odometry>();
        if (pose != NULL)
            gt_file << pose->header.seq << ", " << pose->header.stamp.toSec() << ", " << pose->pose.pose.position.x << ", " << pose->pose.pose.position.y << ", " << pose->pose.pose.position.z << ", " << pose->pose.pose.orientation.x << ", " << pose->pose.pose.orientation.y << ", " << pose->pose.pose.orientation.z << ", " << pose->pose.pose.orientation.w << std::endl;
    }

    gt_file.close();
    bag.close();

    std::string create_frames = " bash /home/" + user_name + "/Myhal_Simulation/raw_bag_files/extract_pcd.sh " + bag_name;
    
    system(create_frames.c_str());
}