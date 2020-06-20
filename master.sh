#!/bin/bash

killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

GUI=false
TOUR="A_tour.bag"
MESSAGE=""
LOADWORLD=""
FILTER=false

while getopts t:m:g:l:f: option
do
case "${option}"
in
t) TOUR=${OPTARG};; # What tour is being used 
m) MESSAGE=${OPTARG};; # A message to add to the log file
g) GUI=${OPTARG};; # do you want to use the gui
l) LOADWORLD=${OPTARG};; # do you want to load a prexisting world or generate a new one
f) FILTER=${OPTARG};; # pointcloud filtering?
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
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs"
touch "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs/log.txt"
echo "Trial Notes: $MESSAGE" >> "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs/log.txt"

sleep 0.1

WORLDFILE="/home/$USER/catkin_ws/src/myhal_simulator/worlds/myhal_sim.world"

if [[ -z $LOADWORLD ]]; then
    rosrun myhal_simulator world_factory
else
    WORLDFILE="/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/logs/myhal_sim.world"
    echo "Loading world $WORLDFILE"
fi

cp $WORLDFILE "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs/"

rosrun jackal_velodyne diagnostics &
roslaunch jackal_velodyne master.launch gui:=$GUI world_name:=$WORLDFILE filter:=$FILTER &
rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" -a -x "/kinect_V2(.*)" # Limiting data to remain under rosbag buffer




