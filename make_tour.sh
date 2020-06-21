#!/bin/bash

killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

roslaunch jackal_velodyne tour_maker.launch
