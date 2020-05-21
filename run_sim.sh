#!/bin/bash


sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster


roslaunch jackal_velodyne full_spawn.launch
