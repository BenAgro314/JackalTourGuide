#!/bin/bash

killall gzclient
killall gzserver
killall roscore
killall rosmaster
killall rviz

LOADWORLD=""
RATE=1

while getopts l:r: option
do
case "${option}"
in
l) LOADWORLD=${OPTARG};; 
r) RATE=${OPTARG};; 
esac
done

BAGPATH="/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/raw_data.bag"

roscore -p $ROSPORT&

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosparam set use_sim_time true

rosbag play -q -l -r $RATE $BAGPATH & 
rosrun dummy_classifier lidarlistener.py