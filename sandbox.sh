#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz

roslaunch jackal_velodyne sandbox.launch
