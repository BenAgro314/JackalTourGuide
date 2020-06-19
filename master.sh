#!/bin/bash

killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

GUI=false
TOUR="A_tour.bag"
MESSAGE=""

while getopts t:m:g: option
do
case "${option}"
in
t) TOUR=${OPTARG};;
m) MESSAGE=${OPTARG};;
g) GUI=${OPTARG};;
esac
done

#1. launch roscore
#2. load parameters 
#3. generate world
#4. launch world 

echo "Running tour: $TOUR"

roscore -p $ROSPORT&

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosparam load src/myhal_simulator/params/common_vehicle_params.yaml
rosparam load src/myhal_simulator/params/animation_params.yaml
rosparam load src/myhal_simulator/params/room_params_V2.yaml
rosparam load src/myhal_simulator/params/scenario_params_V2.yaml
rosparam load src/myhal_simulator/params/plugin_params.yaml
rosparam load src/myhal_simulator/params/model_params.yaml
rosparam set use_sim_time true
rosparam set bag_name $TOUR
t=$(date +'%Y-%m-%d-%H-%M-%S')
rosparam set start_time $t
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t"
touch "/home/$USER/Myhal_Simulation/simulated_runs/$t/log.txt"
echo "Trial Notes: $MESSAGE" >> "/home/$USER/Myhal_Simulation/simulated_runs/$t/log.txt"

sleep 0.1

rosrun myhal_simulator world_factory

rosrun jackal_velodyne diagnostics &
roslaunch jackal_velodyne master.launch gui:=$GUI &
rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" -a -x "/kinect_V2(.*)" # Limiting data to remain under rosbag buffer




