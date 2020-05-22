#!/bin/bash

killall rosmaster
killall roscore

roscore & rosrun jackal_velodyne worldgen3 _num_people:=$1 _num_tables:=$2 && fg
