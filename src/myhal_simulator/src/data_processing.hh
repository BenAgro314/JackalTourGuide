#pragma once

#include <string>
#include <vector>
#include "frame.hh"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include "happily.h"
#include <map>
#include <utility>  



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct FramesAndTraj{
    std::vector<Frame> frames;
    std::vector<TrajPoint> trajectory;
};

class DataProcessor{

    private:

        std::string filename;

        std::string bag_path;

        bool classify;

        rosbag::Bag bag;

        std::vector<std::string> lidar_topics;

        std::string gt_topic;

        FramesAndTraj data;

    public:

        DataProcessor(std::string filename, bool classify);

        void SetTopics(std::string gt_topic, std::vector<std::string> lidar_topics){
            this->gt_topic = gt_topic;
            this->lidar_topics = lidar_topics;
        }

        FramesAndTraj GetData();

        void WriteToPLY();



};


DataProcessor::DataProcessor(std::string filename, bool classify = false){
    this->filename = filename;
    this->classify = classify;

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
        user_name = user;
    } 

    this->bag_path = "/home/" + user_name + "/Myhal_Simulation/simulated_runs/" + filename + "/";
    
}

FramesAndTraj DataProcessor::GetData(){
    std::cout << "Beginning to process bag data\n";
    this->bag.open(this->bag_path + "raw_data.bag", rosbag::bagmode::Read);

    auto query_topics = this->lidar_topics;
    query_topics.push_back(this->gt_topic);

    rosbag::View view(this->bag, rosbag::TopicQuery(query_topics));

    std::vector<Frame> frames;
    std::vector<TrajPoint> trajectory;
   
    
    int count = 0;
    int id = 0;
  
    for (auto msg: view){
        if (msg.getTopic() == this->gt_topic){
        
            auto pose = msg.instantiate<nav_msgs::Odometry>();
            if (pose != nullptr)
                trajectory.push_back(TrajPoint(ignition::math::Pose3d(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z, pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z), pose->header.stamp.toSec()));
            continue;
        } 

        count ++;


        sensor_msgs::PointCloud2::ConstPtr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
        
        if (cloud == nullptr){
            std::cout << "invalid cloud\n";
            continue;
        }
    
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud_ptr);

        int cat = -1;
        if (this->classify){
            

            for (int i =0; i< this->lidar_topics.size(); i++){
                if (msg.getTopic() == this->lidar_topics[i]){
                    cat = i;
                    break;
                }
            }
        }
        if (((count % this->lidar_topics.size())== 1 || this->lidar_topics.size() == 1) && count > 1){
            id++;
        }
    
        if (id >= frames.size()){
            frames.push_back(Frame(false));
            frames[id].SetTime(cloud->header.stamp.toSec());
        }
        
        for (auto point: cloud_ptr->points){
            frames[id].AddPoint(ignition::math::Vector3d(point.x, point.y, point.z), cat);
 
        }
        
    }

    this->bag.close();


    this->data.frames = frames;
    this->data.trajectory = trajectory;
    std::cout << "Successfully processed " << frames.size() << " lidar frames and " << trajectory.size() << " trajectory data points\n";
    return this->data;
}

void DataProcessor::WriteToPLY(){

    std::cout << "Writing data to .ply files\n";
     for (auto frame: this->data.frames){
        frame.WriteToFile(this->bag_path + "bag_frames/");
    }


    happly::PLYData plyOut;
    AddTrajectory(plyOut, this->data.trajectory);
    plyOut.write(this->bag_path + "gt_pose.ply", happly::DataFormat::Binary);
    std::cout << "Sucessfully written data to .ply files\n";
}