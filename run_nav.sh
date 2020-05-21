#!/bin/bash



roslaunch jackal_velodyne amcl_demo.launch
rosrun jackal_velodyne navigation_goals 
rosrun jackal_velodyne points_and_pose _param:=$1
