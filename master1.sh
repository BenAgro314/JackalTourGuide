#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

#1. launch roscore
#2. load parameters 
#3. generate world
#4. launch world 

roscore &

echo num_people: $1 > src/jackal_velodyne/params/simulation_params.yaml
echo num_tables: $2 >> src/jackal_velodyne/params/simulation_params.yaml
echo bag_name: $3 >> src/jackal_velodyne/params/simulation_params.yaml

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosparam load src/jackal_velodyne/params/simulation_params.yaml
sleep 0.1

rosrun jackal_velodyne worldgen3 

roslaunch jackal_velodyne master.launch &
rosbag record -o "data/" -a -x "(.*)/compressedDepth(.*)" #compressed depth images cannot be recorded to rosbag



