#!/bin/bash

killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

LOADWORLD=""
MAPPING=true
GT=false
FILTER=true

while getopts a:l:g:f: option
do
case "${option}"
in
a) MAPPING=${OPTARG};;
l) LOADWORLD=${OPTARG};; 
g) GT=${OPTARG};; 
f) FILTER=${OPTARG};; 
esac
done

roscore -p $ROSPORT&

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

INFILE="/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/raw_data.bag"

rosparam set use_sim_time true

LIDARTOPIC="/hugues_points"
if [ "$GT" == "true" ]; then
    LIDARTOPIC="/velodyne_points"
fi

rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/logs-$LOADWORLD/hugues_test.bag" -a -x "(.*)points(.*)" & # Limiting data to remain under rosbag buffer
roslaunch jackal_velodyne hugues_test.launch mapping:=$MAPPING in_topic:=$LIDARTOPIC filter:=$FILTER &
rosbag play --wait-for-subscribers $INFILE --topics /tf /tf_static $LIDARTOPIC /clock
rosnode kill -a
rosrun myhal_simulator hugues_diagnostics $LOADWORLD $GT $FILTER

