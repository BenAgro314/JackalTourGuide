#pragma once 

#include <string>
#include <vector>
#include <ros/ros.h>
#include "frame.hh"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdio.h>
#include "happily.h"
#include <map>
#include <utility>  
#include "utilities.hh"
#include <fstream>
#include <iostream>



class BagTools{

    private:

        std::string filepath;

        rosbag::Bag bag;


    public:

        BagTools(std::string filepath){
            this->filepath = filepath;
        }


        std::vector<std::vector<double>> TranslationDrift(std::vector<TrajPoint> gt, std::vector<TrajPoint> other){

            std::vector<std::vector<double>> trans_drift;

            double min_time = 10e9;
            double max_time = -10e9;
            for (auto frame: gt){
                min_time = std::min(min_time, frame.time);
                max_time = std::max(max_time, frame.time);
            }

            int last = 0;
            ignition::math::Pose3d last_gt = gt[0].pose;
            double dist = 0;

            //for every frame in amcl pose
            for (auto frame: other){
                double time = frame.time;

                
                if (time < min_time || time > max_time){
                    continue;
                }

                int lower_ind = last;

                // find closest two gt poses 
                while (gt[lower_ind].time > time || gt[lower_ind+1].time < time){
                        
                    if (gt[lower_ind].time > time){
                        lower_ind -=1;
                    } else {
                        lower_ind +=1;
                    }
                }

                last = lower_ind+1;

                auto pose1 = gt[lower_ind].pose;
                    
                auto pose2 = gt[lower_ind+1].pose;

                double t1 = gt[lower_ind].time;
                double t2 = gt[lower_ind+1].time;
                
                // interpolate gt pose to the time of amcl pose
                auto interpolated_pose = utilities::InterpolatePose(time, t1, t2, pose1, pose2);
                dist += (interpolated_pose.Pos()-last_gt.Pos()).Length();
                last_gt = interpolated_pose;

                // compute drift
                trans_drift.push_back({dist, (interpolated_pose.Pos() - frame.pose.Pos()).Length()});
            }

            return trans_drift;
        }


        // may take either nav_msgs::Odometry or geometry_msgs::PoseWithCovarianceStamped
        std::vector<TrajPoint> GetTrajectory(std::string topic){
            std::vector<TrajPoint> trajectory;
            rosbag::Bag bag;
            bag.open(this->filepath + "raw_data.bag", rosbag::bagmode::Read);
            rosbag::View view(bag, rosbag::TopicQuery({topic}));

            for (auto msg: view){
                
                if (msg.getTopic() == topic){
                
                    auto pose = msg.instantiate<nav_msgs::Odometry>();
                    if (pose != nullptr) {
                        trajectory.push_back(TrajPoint(ignition::math::Pose3d(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z, pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z), pose->header.stamp.toSec()));
                    } else{
                        auto p = msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
                        trajectory.push_back(TrajPoint(ignition::math::Pose3d(p->pose.pose.position.x, p->pose.pose.position.y, p->pose.pose.position.z, p->pose.pose.orientation.w, p->pose.pose.orientation.x, p->pose.pose.orientation.y, p->pose.pose.orientation.z), p->header.stamp.toSec()));
                    }
                    
                } 
            }


            return trajectory;
            bag.close();
        }

};