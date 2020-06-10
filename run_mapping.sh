#!/bin/bash
# used for mapping a new area
killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster
roslaunch jackal_velodyne mapping.launch
