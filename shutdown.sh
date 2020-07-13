#!/bin/bash

TIME=$(rosparam get start_time)
FILTER=$(rosparam get filter_status)
REPEAT=$(rosparam get repeat)


rosnode kill -a
killall rosmaster
killall gzserver
killall gzclient

sleep 0.5
echo "Running data_processing.py"
rosrun dashboard data_processing.py $TIME

rosnode kill -a
killall rosmaster
killall gzserver
killall gzclient
killall roscore

if [ "$REPEAT" -lt "1" ]
then
    echo "Done repeating, shutting down"
    exit 1
fi

echo "$REPEAT trials remaining"

#/bin/bash /home/$USER/catkin_ws/master.sh -t short_test


#rosrun myhal_simulator process_bag $TIME
#echo "Running bag diagnostics"
#rosrun myhal_simulator bag_diagnostics $TIME $FILTER
#echo "Bag diagnostics completed"