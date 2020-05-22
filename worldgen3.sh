#!/bin/bash
#generates new world using worldgen3 program

sudo killall gzserver
sudo killall gzclient
sudo killall rviz

rosrun jackal_velodyne worldgen3 _num_people:=$1 _num_tables:=$2
roslaunch jackal_velodyne worldgen3.launch
