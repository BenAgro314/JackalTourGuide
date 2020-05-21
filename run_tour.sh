#!/bin/bash

bash run_nav.sh &
rosrun jackal_velodyne navigation_goals 
rosrun jackal_velodyne points_and_pose _param:=$1
