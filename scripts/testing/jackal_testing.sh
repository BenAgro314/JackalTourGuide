#!/bin/bash


GUI=false # -v flag
FILTER=false # -f flag
MAPPING=false # -m flag
GTCLASS=false # -g flag 

while getopts :v:fmg option
do
case "${option}"
in
v) GUI=true;; # using gui?
f) FILTER=true;; # pointcloud filtering?
m) MAPPING=true;; # use gmapping?
g) GTCLASS=true;; # are we using ground truth classifications, or online_classifications
esac
done

killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

c_method="ground_truth"

if [ "$FILTER" = false ] ; then
    c_method="none"
    GTCLASS=false
else
    if [ "$GTCLASS" = false ] ; then
        c_method="online_predictions"
    else 
        c_method="ground_truth"
    fi
fi

export GTCLASSIFY=$GTCLASS

roscore -p $ROSPORT&

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosparam load src/myhal_simulator/params/common_vehicle_params.yaml

rosparam set gt_class $GTCLASS
rosparam set localization_test false
rosparam set class_method $c_method
rosparam set use_sim_time true
rosparam set filter_status $FILTER
rosparam set gmapping_status $MAPPING

roslaunch jackal_velodyne jackal_testing.launch gui:=$GUI filter:=$FILTER mapping:=$MAPPING gt_classify:=$GTCLASS 

