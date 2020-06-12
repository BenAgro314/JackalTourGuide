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