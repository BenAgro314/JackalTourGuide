#!/bin/bash

myInvocation="$(printf %q "$BASH_SOURCE")$((($#)) && printf ' %q' "$@")"
t=$(date +'%Y-%m-%d-%H-%M-%S')

echo "Folder Name: $t"


GUI=false
TOUR="A_tour"
MESSAGE=""
LOADWORLD=""
FILTER=false
MAPPING=false
CLASS=true
REPEAT=1

while getopts t:m:g:l:f:a:c:r: option
do
case "${option}"
in
t) TOUR=${OPTARG};; # What tour is being used 
m) MESSAGE=${OPTARG};; # A message to add to the log file
g) GUI=${OPTARG};; # do you want to use the gui
l) LOADWORLD=${OPTARG};; # do you want to load a prexisting world or generate a new one
f) FILTER=${OPTARG};; # pointcloud filtering?
a) MAPPING=${OPTARG};; # use gmapping?
c) CLASS=${OPTARG};; # are we classifying points at all?
r) REPEAT=${OPTARG};; # how many times to loop this trial?
esac
done

if [ "$REPEAT" -lt "1" ]
then
    echo "Done repeating, shutting down"
    exit 1
fi

echo "$REPEAT trials remaining"

sleep 1

killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

export CLASSIFY=$CLASS

c_method="ground_truth"

# ...do something interesting...
if [ "$CLASS" = false ] ; then
    FILTER=false
    c_method="none"
fi


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
rosparam set localization_test false
rosparam set class_method $c_method
# rosparam set repeat $REPEAT
rosparam set use_sim_time true
rosparam set tour_name $TOUR
rosparam set classify $CLASSIFY
rosparam load src/myhal_simulator/tours/$TOUR/config.yaml
rosparam set start_time $t
rosparam set filter_status $FILTER
rosparam set gmapping_status $MAPPING

mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t"
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t"
LOGFILE="/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/log.txt"
PARAMFILE="/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/params.yaml"
touch $LOGFILE
echo -e "Trial Notes: $MESSAGE\nFILTER: $FILTER\nMAPPING $MAPPING" >> $LOGFILE
echo -e "Command used: $myInvocation" >> $LOGFILE
echo -e "\nPointcloud filter params: \n" >> $LOGFILE
echo -e "$(cat /home/$USER/catkin_ws/src/jackal_velodyne/launch/include/pointcloud_filter.launch)" >> $LOGFILE
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
rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" /clock /shutdown_signal /velodyne_points /move_base/local_costmap/costmap /move_base/global_costmap/costmap /ground_truth/state /map /move_base/NavfnROS/plan /amcl_pose /tf /tf_static /move_base/result /tour_data /optimal_path &
rosrun jackal_velodyne diagnostics &
roslaunch jackal_velodyne master.launch gui:=$GUI world_name:=$WORLDFILE filter:=$FILTER mapping:=$MAPPING
sleep 0.5
echo "Running data_processing.py"
rosrun dashboard data_processing.py $t




