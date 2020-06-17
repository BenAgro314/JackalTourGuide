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
rosparam set publish_points false # activly catagorize lidar points?
rosparam set publish_ply false # activly catagorize lidar points and publish to .ply?
rosparam set record_objects true # recording model positions to .ply for post processing?
rosparam set publish_navigation false # republish lidar points with actors removed for navigation?
t=$(date +'%Y-%m-%d-%H-%M-%S')
rosparam set start_time $t
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t"
touch "/home/$USER/Myhal_Simulation/simulated_runs/$t/log.txt"

sleep 0.1

rosrun jackal_velodyne diagnostics &
roslaunch jackal_velodyne jackal_testing.launch &
rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" -a -x "/kinect_V2(.*)" # Limiting data to remain under rosbag buffer