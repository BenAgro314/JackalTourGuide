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


    command = "mkdir /home/" + user_name + "/Myhal_Simulation/simulated_runs/" + filename + "/frames";
    system(command.c_str());
    

    // extract topic: /ground_truth/state type: nav_msgs/Odometry    
    std::vector<std::string> topics = {"/ground_truth/state", "/ground_points", "/chair_points", "/moving_actor_points", "/still_actor_points","/table_points", "/wall_points"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    gt_file << "time, pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w\n";


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
        std::array<double, 3> color;
        count ++;
        if (msg.getTopic() == "/ground_points"){
            color = {0,0,0}; //black
        } else if (msg.getTopic() == "/chair_points"){
            color = {1,0,1}; //purple
        } else if (msg.getTopic() == "/moving_actor_points"){
            color = {1,0,0}; //red
        } else if (msg.getTopic() == "/still_actor_points"){
            color = {1,0.5,0}; //orange
        } else if (msg.getTopic() == "/table_points"){
            color = {0,1,0};  //green
        } else if (msg.getTopic() == "/wall_points"){
            color = {0,1,1}; //cyan
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
            //std::cout << std::endl; 
        }
        //std::cout << cloud->header.stamp.toSec() << " " << msg.getTopic() << std::endl;
       
        if (id >= frames.size()){
            std::vector<std::array<double, 3>> pts;
            std::vector<std::array<double, 3>> colors;
            std::vector<std::vector<std::array<double, 3>>> points;
            points.push_back(pts);
            points.push_back(colors);
            std::pair<double,std::vector<std::vector<std::array<double, 3>>>> frame;
            frame.first = cloud->header.stamp.toSec();
            frame.second = points;
            frames.push_back(frame);
        }
        
        for (auto point: cloud_ptr->points){
            frames[id].second[0].push_back({point.x, point.y, point.z});
            frames[id].second[1].push_back(color);
        }
        
        
    }

    gt_file.close();
    bag.close();

    
    for (auto frame: frames){
        
        happly::PLYData plyOut;
        std::vector<std::array<double, 3>> points = frame.second[0];
        std::vector<std::array<double, 3>> colors = frame.second[1];
        double time = frame.first;

        // Add mesh data (elements are created automatically)
        plyOut.addVertexPositions(points);
        plyOut.addVertexColors(colors);
        // Write the object to file
        

        plyOut.write("/home/" + user_name + "/Myhal_Simulation/simulated_runs/" +  filename + "/frames/" + std::to_string(time) + ".ply", happly::DataFormat::ASCII);
      
    }


    
}