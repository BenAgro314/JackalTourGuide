#!/bin/bash
# used for mapping a new area
sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster
roslaunch jackal_velodyne mapping.launch
