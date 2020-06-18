TIME=$(rosparam get start_time)

rosnode kill -a
killall rosmaster
killall gzserver
killall gzclient

sleep 0.5
rosrun myhal_simulator process_bag $TIME
rosrun myhal_simulator process_bag $TIME true