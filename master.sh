#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

roslaunch jackal_velodyne master.launch num_people:=$1 num_tables:=$2 bag_name:=$3 &
rosbag record -a -x "/kinect_V2(.*)" #TODO: fix this line



