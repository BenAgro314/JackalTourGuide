#!/bin/bash
#generates new world using worldgen3 program

#1. load parameters 
#2. generate world
#3. launch world 

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

bash shutdown.sh

roscore &
echo num_people: $1 > src/jackal_velodyne/params/simulation_params.yaml
echo num_tables: $2 >> src/jackal_velodyne/params/simulation_params.yaml
until rostopic list; do sleep 0.5; done
rosparam load src/jackal_velodyne/params/simulation_params.yaml
rosrun jackal_velodyne worldgen3 #_num_people:=$1 _num_tables:=$2
roslaunch jackal_velodyne worldgen3.launch
