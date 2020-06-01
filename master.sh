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

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosparam load src/myhal_simulator/params/animation_params.yaml
rosparam load src/myhal_simulator/params/room_params.yaml
rosparam load src/myhal_simulator/params/scenario_params.yaml
rosparam load src/myhal_simulator/params/plugin_params.yaml
rosparam load src/myhal_simulator/params/model_params.yaml
sleep 0.1

rosrun myhal_simulator world_factory

roslaunch jackal_velodyne master.launch &
rosbag record -o "data/" -a -x "(.*)/compressedDepth(.*)" #compressed depth images cannot be recorded to rosbag



