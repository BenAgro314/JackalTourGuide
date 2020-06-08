#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include <rosbag/bag.h>
#include <iostream>
#include <fstream>




using namespace std;


int num_frames = 0;
int max_frames = 500;
//rosbag::Bag bag1;
rosbag::Bag bag2;
ofstream file;

double x,y,z,qx,qy,qz,qw;

void GroundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg){
	
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	z = msg->pose.pose.position.z;;
	
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz= msg->pose.pose.orientation.z;
	qw= msg->pose.pose.orientation.w;

}

void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	
	//ROS_INFO("frame: %d", num_frames);
	
	
	file << num_frames << ", " << ros::Time::now() << ", " << x << ", " << y << ", " << z << ", " << qx << ", " << qy << ", " << qz << ", " << qw << endl;
	bag2.write("lidar_points", ros::Time::now(), msg);
	num_frames++;
	
	if (num_frames >= max_frames){
		ros::shutdown();
		ROS_INFO("DONE");
	
		//bag1.close();
		bag2.close();
		file.close();
	}
}

int main(int argc, char ** argv){
	
	ros::init(argc, argv, "ground_truth_monitor");

	ros::NodeHandle nh;
	
	//bag1.open("test_pose.bag", rosbag::bagmode::Write);
	
	
	string param("default_name");
	/*
	if (!nh.getParam("param", param)){
		param = "default_name";
	}
	*/
	cout << "FILENAME: " << param << endl;
	
	string bag_path = "./data/" + param + "_lidar.bag";
	string file_path = "./data/" + param + "_pose.csv";
	
	bag2.open(bag_path, rosbag::bagmode::Write);
	file.open(file_path);
	file << "frame, time, position x, position y, position z, orientation x, orientation y, orientation z, orientation w" << endl;
	
	
	ros::Time begin = ros::Time::now();
	
	ros::Subscriber pose_sub = nh.subscribe("ground_truth/state", 1000, GroundTruthCallback);
	ros::Subscriber lidar_sub = nh.subscribe("velodyne_points", 1000, LidarCallback);
	ros::spin();
		

	
	
	return 0;

	
}
