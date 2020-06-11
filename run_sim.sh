#!/bin/bash


killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

roscore &

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosparam load src/myhal_simulator/params/common_vehicle_params.yaml
rosparam set use_sim_time true
t=$(date +'%Y-%m-%d-%s')
rosparam set start_time $t
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t"
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t/frames"

sleep 0.1

roslaunch jackal_velodyne jackal_testing.launch &
rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" -a -x "/kinect_V2(.*)" # Limiting data to remain under rosbag buffer