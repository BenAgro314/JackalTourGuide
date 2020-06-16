#!/bin/bash

killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

roscore &

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosparam load src/myhal_simulator/params/animation_params.yaml
rosparam load src/myhal_simulator/params/room_params_V2.yaml
rosparam load src/myhal_simulator/params/scenario_params_V2.yaml
rosparam load src/myhal_simulator/params/plugin_params.yaml
rosparam load src/myhal_simulator/params/model_params.yaml
rosparam load src/myhal_simulator/params/common_vehicle_params.yaml

sleep 0.1

rosrun myhal_simulator world_factory

roslaunch jackal_velodyne myhal_sim_test.launch
