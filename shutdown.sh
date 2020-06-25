TIME=$(rosparam get start_time)
FILTER=$(rosparam get filter_status)

rosnode kill -a
killall rosmaster
killall gzserver
killall gzclient

sleep 0.5
rosrun myhal_simulator process_bag $TIME
echo "Running bag diagnostics"
rosrun myhal_simulator bag_diagnostics $TIME $FILTER
echo "Bag diagnostics completed"