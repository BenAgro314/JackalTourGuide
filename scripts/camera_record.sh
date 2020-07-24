#!/bin/bash

rosbag record -o "/home/$USER/Myhal_Simulation/" -e "(.*)image_raw(.*)" -x "(.*)compressedDepth(.*)"

