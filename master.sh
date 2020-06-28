#!/bin/bash

myInvocation="$(printf %q "$BASH_SOURCE")$((($#)) && printf ' %q' "$@")"
t=$(date +'%Y-%m-%d-%H-%M-%S')

echo "Folder Name: $t"
sleep 1

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
rosparam set tour_name $TOUR
rosparam load src/myhal_simulator/tours/$TOUR/config.yaml
rosparam set start_time $t
rosparam set filter_status $FILTER
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t"
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t"
touch "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/log.txt"
echo -e "Trial Notes: $MESSAGE\nFILTER: $FILTER\nMAPPING $MAPPING" >> "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/log.txt"
echo -e "Command used: $myInvocation" >> "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/log.txt"

sleep 0.1

WORLDFILE="/home/$USER/catkin_ws/src/myhal_simulator/worlds/myhal_sim.world"

if [[ -z $LOADWORLD ]]; then
    rosrun myhal_simulator world_factory
else
    WORLDFILE="/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/logs-$LOADWORLD/myhal_sim.world"
    echo "Loading world $WORLDFILE"
fi

cp $WORLDFILE "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/"

#rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" -a -x "/kinect_V2(.*)" & # Limiting data to remain under rosbag buffer
rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" /clock /velodyne_points /move_base/local_costmap/costmap /move_base/global_costmap/costmap /ground_truth/state /map /move_base/NavfnROS/plan /amcl_pose /tf /tf_static /move_base/result /tour_data &
rosrun jackal_velodyne diagnostics &
roslaunch jackal_velodyne master.launch gui:=$GUI world_name:=$WORLDFILE filter:=$FILTER mapping:=$MAPPING





