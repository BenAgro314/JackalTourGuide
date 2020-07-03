#!/bin/bash

killall gzclient
killall gzserver
killall roscore
killall rosmaster
killall rviz

roslaunch world_generator test.launch