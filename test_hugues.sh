#!/bin/bash

killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

LOADWORLD=""
MAPPING = true

while getopts a:l: option
do
case "${option}"
in
a) MAPPING=${OPTARG};;
l) LOADWORLD=${OPTARG};; 
esac
done

roscore -p $ROSPORT&

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

INFILE="/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/raw_data.bag"

rosparam set use_sim_time true

rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/logs-$LOADWORLD/hughes_test.bag" -a -x "/hugues_points" & # Limiting data to remain under rosbag buffer
roslaunch jackal_velodyne hugues_test.launch mapping:=$MAPPING &
rosbag play --wait-for-subscribers $INFILE --topics /tf /tf_static /hugues_points /clock
rosnode kill -a

