#!/bin/bash

killall gzserver
killall gzclient
killall rviz

roslaunch jackal_velodyne sandbox.launch
