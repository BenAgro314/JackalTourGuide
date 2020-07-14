#!/bin/bash

TIME=$(rosparam get start_time)
FILTER=$(rosparam get filter_status)
# REPEAT=$(rosparam get repeat)


rosnode kill -a
killall rosmaster
killall gzserver
killall gzclient

