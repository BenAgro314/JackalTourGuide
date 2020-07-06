#!/bin/bash

killall gzclient
killall gzserver
killall roscore
killall rosmaster
killall rviz

roslaunch dummy_classifier test.launch