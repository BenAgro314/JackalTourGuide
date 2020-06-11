#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <vector>
#include "happily.h"
#include <map>
#include <utility>  
#include "frame.hh"


#define NUM_TOPICS 6

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv){
    if (argc == 1){
        std::cout << "must input bag name\n";
        return 0;
    }

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
      user_name = user;
    } 

    std::string time_name = argv[1];

    rosbag::Bag bag;
    
    bag.open("/home/" + user_name + "/Myhal_Simulation/simulated_runs/" + time_name + "/raw_data.bag", rosbag::bagmode::Read);
    std::ofstream gt_file;
    gt_file.open("/home/" + user_name + "/Myhal_Simulation/simulated_runs/" +  time_name + "/gt_pose.csv");


    std::string command = "mkdir /home/" + user_name + "/Myhal_Simulation/simulated_runs/" + time_name + "/bag_frames";
    system(command.c_str());
    
    // extract topic: /ground_truth/state type: nav_msgs/Odometry    
    std::vector<std::string> topics = {"/ground_truth/state", "/ground_points", "/chair_points", "/moving_actor_points", "/still_actor_points","/table_points", "/wall_points"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    gt_file << "time, pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w\n";

    std::vector<Frame> F;
   

    // vector where the ith index is the ith lidar frame (pair of the time, and vector of the points and their catagory)
    std::vector<std::pair<double,std::vector<std::vector<std::array<double, 3>>>>> frames;
    
    int count = 0;
    int id = 0;
  
    for (auto msg: view){

       
        if (msg.getTopic() == "/ground_truth/state"){
        
            auto pose = msg.instantiate<nav_msgs::Odometry>();
            if (pose != nullptr)
                gt_file << pose->header.stamp.toSec() << ", " << pose->pose.pose.position.x << ", " << pose->pose.pose.position.y << ", " << pose->pose.pose.position.z << ", " << pose->pose.pose.orientation.x << ", " << pose->pose.pose.orientation.y << ", " << pose->pose.pose.orientation.z << ", " << pose->pose.pose.orientation.w << std::endl;
            continue;
        } 
        char cat;
        count ++;
        if (msg.getTopic() == "/ground_points"){
            cat = 'g';
        } else if (msg.getTopic() == "/chair_points"){
            cat = 'c';
        } else if (msg.getTopic() == "/moving_actor_points"){
            cat = 'm';
        } else if (msg.getTopic() == "/still_actor_points"){
            cat = 's';
        } else if (msg.getTopic() == "/table_points"){
            cat = 't';
        } else if (msg.getTopic() == "/wall_points"){
            cat = 'w';
        }

        

        sensor_msgs::PointCloud2::ConstPtr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
        
        if (cloud == nullptr){
            std::cout << "invalid cloud\n";
            continue;
        }
    
        
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud_ptr);

        if (count % NUM_TOPICS== 1 && count > 1){
            id++;
        }
       
        if (id >= F.size()){
            F.push_back(Frame(false));
            F[id].SetTime(cloud->header.stamp.toSec());
        }
        
        for (auto point: cloud_ptr->points){
            F[id].AddPoint(ignition::math::Vector3d(point.x, point.y, point.z), cat);
        }
        
        
    }

    gt_file.close();
    bag.close();

    for (auto frame: F){
        frame.WriteToFile("/home/" + user_name + "/Myhal_Simulation/simulated_runs/" +  time_name + "/bag_frames/");
    }

    
}