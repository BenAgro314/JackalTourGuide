#!/bin/bash
#generates new world using worldgen3 program

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

roscore &
echo num_people: $1 > src/jackal_velodyne/params/simulation_params.yaml
echo num_tables: $2 >> src/jackal_velodyne/params/simulation_params.yaml
sleep 1
rosparam load src/jackal_velodyne/params/simulation_params.yaml
rosrun jackal_velodyne worldgen3 
roslaunch jackal_velodyne worldgen3.launch
