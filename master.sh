#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

echo num_people: $1 > src/jackal_velodyne/params/simulation_params.yaml
echo num_tables: $2 >> src/jackal_velodyne/params/simulation_params.yaml
echo bag_name: $3 >> src/jackal_velodyne/params/simulation_params.yaml
roslaunch jackal_velodyne master.launch &
rosbag record -a -x "/kinect_V2(.*)" #TODO: fix this line



