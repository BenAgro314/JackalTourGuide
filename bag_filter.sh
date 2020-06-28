#!/bin/bash

killall roscore

LOADWORLD=""

while getopts l: option
do
case "${option}"
in
l) LOADWORLD=${OPTARG};; 
esac
done

BAGPATH="/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/raw_data.bag"
NEWBAGPATH="/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/logs-$LOADWORLD/filtered.bag"

roscore -p $ROSPORT&

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosbag filter $BAGPATH $NEWBAGPATH "topic == '/tf' or topic == '/clock' or topic == '/map' or topic == '/velodyne_points' or topic == '/tf_static' or topic =='move_base/NavfnROS/plan'"