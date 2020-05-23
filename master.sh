#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

roscore &
echo num_people: $1 > src/jackal_velodyne/params/simulation_params.yaml
echo num_tables: $2 >> src/jackal_velodyne/params/simulation_params.yaml
echo bag_name: $3 >> src/jackal_velodyne/params/simulation_params.yaml
sleep 1 #we wait until roscore starts 
rosparam load src/jackal_velodyne/params/simulation_params.yaml
sleep 1 #wait for parameters to load
rosrun jackal_velodyne worldgen3 
roslaunch jackal_velodyne master.launch &
rosbag record -o "data/" -a -x "(.*)/compressedDepth(.*)" #compressed depth images cannot be recorded to rosbag



