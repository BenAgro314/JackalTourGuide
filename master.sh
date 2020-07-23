#!/bin/bash

myInvocation="$(printf %q "$BASH_SOURCE")$((($#)) && printf ' %q' "$@")"
t=$(date +'%Y-%m-%d-%H-%M-%S')

echo "Folder Name: $t"


GUI=false # -v flag
TOUR="A_tour" # -t (arg) flag
LOADWORLD="" # -l (arg) flag
FILTER=false # -f flag
MAPPING=false # -m flag
GTCLASS=false # -g flag 

while getopts t:vl:fmg option
do
case "${option}"
in
t) TOUR=${OPTARG};; # What tour is being used 
v) GUI=true;; # using gui?
l) LOADWORLD=${OPTARG};; # do you want to load a prexisting world or generate a new one
f) FILTER=true;; # pointcloud filtering?
m) MAPPING=true;; # use gmapping?
g) GTCLASS=true;; # are we using ground truth classifications, or online_classifications
esac
done

echo -e "TOUR: $TOUR\nGUI: $GUI\nLOADWORLD: $LOADWORLD\nFILTER: $FILTER\nMAPPING: $MAPPING\nGTCLASS: $GTCLASS"

sleep 1

killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

c_method="ground_truth"

if [ "$FILTER" = false ] ; then
    c_method="none"
    GTCLASS=false
else
    if [ "$GTCLASS" = false ] ; then
        c_method="online_predictions"
    else 
        c_method="ground_truth"
    fi
fi

export GTCLASSIFY=$GTCLASS


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
rosparam load src/myhal_simulator/tours/$TOUR/config.yaml

rosparam set gt_class $GTCLASS
rosparam set localization_test false
rosparam set class_method $c_method
rosparam set use_sim_time true
rosparam set tour_name $TOUR
rosparam set start_time $t
rosparam set filter_status $FILTER
rosparam set gmapping_status $MAPPING
rosparam set camera_pos "[false, false, true , false]" # represents [top left, top right, bot left, bot right]

mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t"
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t"
LOGFILE="/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/log.txt"
PARAMFILE="/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/params.yaml"
PCLFILE="/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/pcl.txt"
touch $LOGFILE
echo -e "Command used: $myInvocation" >> $LOGFILE
echo -e "\nPointcloud filter params: \n" >> $LOGFILE
echo -e "TOUR: $TOUR\nGUI: $GUI\nLOADWORLD: $LOADWORLD\nFILTER: $FILTER\nMAPPING: $MAPPING\nGTCLASS: $GTCLASS"  >> $LOGFILE
echo -e "$(cat /home/$USER/catkin_ws/src/jackal_velodyne/launch/include/pointcloud_filter2.launch)" >> $PCLFILE

echo -e "$(cat /home/$USER/catkin_ws/src/myhal_simulator/params/room_params_V2.yaml)" > $PARAMFILE
echo -e "\n" >> $PARAMFILE
echo -e "$(cat /home/$USER/catkin_ws/src/myhal_simulator/params/scenario_params_V2.yaml)" >> $PARAMFILE
echo -e "\n" >> $PARAMFILE
echo -e "$(cat /home/$USER/catkin_ws/src/myhal_simulator/params/plugin_params.yaml)" >> $PARAMFILE
echo -e "\n" >> $PARAMFILE
echo -e "tour_name: $TOUR" >> $PARAMFILE

sleep 0.1

WORLDFILE="/home/$USER/catkin_ws/src/myhal_simulator/worlds/myhal_sim.world"

if [[ -z $LOADWORLD ]]; then
    rosrun myhal_simulator world_factory
    rosparam set load_world "none"
else
    WORLDFILE="/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/logs-$LOADWORLD/myhal_sim.world"
    rosparam set load_world $LOADWORLD
    echo "Loading world $WORLDFILE"
fi

cp $WORLDFILE "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/"

#rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" -a -x "/kinect_V2(.*)" & # Limiting data to remain under rosbag buffer
rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" /clock /shutdown_signal /velodyne_points /move_base/local_costmap/costmap /move_base/global_costmap/costmap /ground_truth/state /map /move_base/NavfnROS/plan /amcl_pose /tf /tf_static /move_base/result /tour_data /optimal_path /classified_points &
rosrun jackal_velodyne diagnostics &
roslaunch jackal_velodyne master.launch gui:=$GUI world_name:=$WORLDFILE filter:=$FILTER mapping:=$MAPPING gt_classify:=$GTCLASS
sleep 0.5
echo "Running data_processing.py"
rosrun dashboard data_processing.py $t




