rosnode kill -a 
killall gzclient
killall gzserver
killall roscore
killall rosmaster
killall rviz
killall -s SIGKILL master.sh
killall -s SIGKILL roslaunch
