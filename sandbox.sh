#!/bin/bash

killall gzserver
killall gzclient
killall rviz

rosparam set use_sim_time true
roslaunch jackal_velodyne sandbox.launch 
