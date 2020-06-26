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
#include "utilities.hh"

struct FrameData{
    std::vector<Frame> frames;
    std::vector<TrajPoint> trajectory;
};

class BagProcessor{

    private:

        std::string filepath;

        bool translate;

        rosbag::Bag bag;

        std::string gt_topic;

        std::string lidar_topic;

        FrameData data;

    public:

        BagProcessor(std::string filepath, std::string gt_topic, std::string lidar_topic, bool translate):
        filepath(filepath), gt_topic(gt_topic), lidar_topic(lidar_topic), translate(translate){};

        FrameData GetData(){

            std::cout << "Beginning to process bag data\n";
            this->bag.open(this->filepath + "raw_data.bag", rosbag::bagmode::Read);

            std::vector<Frame> frames;
            std::vector<TrajPoint> trajectory;

            if (this->translate){

                double min_traj_time = 10e9;
                double max_traj_time = -10e9;
                int last = 0;

                std::vector<std::string> query_topics = {this->gt_topic};
                rosbag::View gt_view(this->bag, rosbag::TopicQuery(query_topics));
                for (auto msg: gt_view){
                    if (msg.getTopic() == this->gt_topic){
                    
                        auto pose = msg.instantiate<nav_msgs::Odometry>();
                        if (pose != nullptr) {
                            trajectory.push_back(TrajPoint(ignition::math::Pose3d(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z, pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z), pose->header.stamp.toSec()));
                            min_traj_time = std::min(min_traj_time, pose->header.stamp.toSec());
                            max_traj_time = std::max(max_traj_time, pose->header.stamp.toSec());
                        }
                    } 
                }

                query_topics = {this->lidar_topic};
                rosbag::View lidar_view(this->bag, rosbag::TopicQuery(query_topics));
                for (auto msg: lidar_view){
                    sensor_msgs::PointCloud2::ConstPtr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
                    pcl::PCLPointCloud2 pcl_pc2;
                    pcl_conversions::toPCL(*cloud,pcl_pc2);
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_ptr);

                    

                    // find gt_pose interpolation 

                    double time = cloud->header.stamp.toSec();

                    if (time < min_traj_time || time > max_traj_time){
                        std::cout << "Dropping frame at time " << time << " because it is out of range of the trajectory times\n";
                        continue;
                    }

                    frames.push_back(Frame(false));
                    

                    int lower_ind = last;
        
                    while (trajectory[lower_ind].time > time || trajectory[lower_ind+1].time < time){
                        
                        if (trajectory[lower_ind].time > time){
                            lower_ind -=1;
                        } else {
                            lower_ind +=1;
                        }
                    }
                    
                    last = lower_ind+1;
                
                    auto pose1 = trajectory[lower_ind].pose;
                    
                    auto pose2 = trajectory[lower_ind+1].pose;

                    auto tf_trans = ignition::math::Pose3d(0,0,0.539,1,0,0,0) + ignition::math::Pose3d(0,0,0,1,0,0,0) + ignition::math::Pose3d(0,0,0.0377,1,0,0,0);

                    pose1+=tf_trans;
                    pose2+=tf_trans;
                    double t1 = trajectory[lower_ind].time;
                    double t2 = trajectory[lower_ind+1].time;

                    auto translation = utilities::InterpolatePose(time, t1, t2, pose1, pose2);

                    frames.back().SetTime(time);
                    for (auto point: cloud_ptr->points){
                        auto trans_pt = translation.CoordPositionAdd(ignition::math::Vector3d(point.x, point.y, point.z));
                        frames.back().AddPoint(trans_pt, point.intensity);
                    }
                }

            } else{
                std::vector<std::string> query_topics = {this->gt_topic, this->lidar_topic};
                rosbag::View view(this->bag, rosbag::TopicQuery(query_topics));

                for (auto msg: view){
                    if (msg.getTopic() == this->gt_topic){
                    
                        auto pose = msg.instantiate<nav_msgs::Odometry>();
                        if (pose != nullptr) {
                            trajectory.push_back(TrajPoint(ignition::math::Pose3d(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z, pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z), pose->header.stamp.toSec()));
                        }
                        continue;
                    } 

                    sensor_msgs::PointCloud2::ConstPtr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
                    if (cloud == nullptr){
                        std::cout << "invalid cloud\n";
                        continue;
                    }
            
                    pcl::PCLPointCloud2 pcl_pc2;
                    pcl_conversions::toPCL(*cloud,pcl_pc2);
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_ptr);

                    frames.push_back(Frame(false));
                    frames.back().SetTime(cloud->header.stamp.toSec());
                    for (auto point: cloud_ptr->points){
                        frames.back().AddPoint(ignition::math::Vector3d(point.x, point.y, point.z), point.intensity);
                    }
                    
                }

            }

            this->bag.close();

            this->data.frames = frames;
            this->data.trajectory = trajectory;
            std::cout << "Successfully processed " << frames.size() << " lidar frames and " << trajectory.size() << " trajectory data points\n";
            return data;
        }

        void WriteToPLY(){
            std::cout << "Writing data to .ply files\n";

            std::string path;
            if (this->translate){
                path = this->filepath + "classified_transformed_frames/";
            } else{
                path = this->filepath + "classified_frames/";
            }

            std::string command = "mkdir " + path;
            //std::cout << command << std::endl;

            system(command.c_str());
            
   
            for (auto frame: this->data.frames){
                frame.WriteToFile(path);
            }

            happly::PLYData plyOut;
            AddTrajectory(plyOut, this->data.trajectory);
            plyOut.write(this->filepath + "gt_pose.ply", happly::DataFormat::Binary);
            std::cout << "Sucessfully written data to .ply files\n";
        }


};

