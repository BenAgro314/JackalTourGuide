TIME=$(rosparam get start_time)

rosnode kill -a
killall rosmaster
killall gzserver
killall gzclient

sleep 0.5
rosrun myhal_simulator process_bag $TIME
echo "Running bag diagnostics"
rosrun myhal_simulator bag_diagnostics $TIME
echo "Bag diagnostics completed"