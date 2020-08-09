#!/bin/bash

killall gzserver
killall gzclient
killall rviz

roscore -p $ROSPORT&

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosparam load src/myhal_simulator/params/default_params/common_vehicle_params.yaml
rosparam load src/myhal_simulator/params/default_params/animation_params.yaml
rosparam load src/myhal_simulator/params/default_params/room_params_V2.yaml
rosparam load src/myhal_simulator/params/default_params/scenario_params_V2.yaml
rosparam load src/myhal_simulator/params/default_params/plugin_params.yaml
rosparam load src/myhal_simulator/params/default_params/model_params.yaml

export GTCLASSIFY=0

rosparam set use_sim_time true
roslaunch jackal_velodyne sandbox.launch 
