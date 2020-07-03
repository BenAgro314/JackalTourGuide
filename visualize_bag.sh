#!/bin/bash

killall roscore
killall rviz 
killall rosmaster

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

#rviz -d "/home/$USER/catkin_ws/src/jackal_velodyne/launch/include/visualize.rviz" &
roslaunch jackal_velodyne visualize.launch &
sleep 4
rosbag play -l -r $RATE $BAGPATH