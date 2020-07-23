#!/bin/bash


killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

GUI=false
TOUR="A_tour"
MESSAGE=""
LOADWORLD=""
FILTER=false
MAPPING=false

while getopts t:m:g:l:f:a: option
do
case "${option}"
in
t) TOUR=${OPTARG};; # What tour is being used 
m) MESSAGE=${OPTARG};; # A message to add to the log file
g) GUI=${OPTARG};; # do you want to use the gui
l) LOADWORLD=${OPTARG};; # do you want to load a prexisting world or generate a new one
f) FILTER=${OPTARG};; # pointcloud filtering?
a) MAPPING=${OPTARG};; # use gmapping?
esac
done

roscore &

until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosparam load src/myhal_simulator/params/common_vehicle_params.yaml
rosparam set use_sim_time true
t=$(date +'%Y-%m-%d-%H-%M-%S')
rosparam set start_time $t
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t"
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t"
touch "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/log.txt"
echo $2 >> "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/log.txt"
 
sleep 0.1

roslaunch jackal_velodyne jackal_testing.launch mapping:=$MAPPING filter:=$FILTER &
rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" -a -x "/kinect_V2(.*)" # Limiting data to remain under rosbag buffer
