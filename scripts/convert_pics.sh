#!/bin/bash


NAME='camera'
PREFIX='/tmp/'

while getopts n:p: option
do
case "${option}"
in
p) PREFIX=${OPTARG};;
n) NAME=${OPTARG};;
esac
done

ffmpeg -r 30 -pattern_type glob -i "${PREFIX}${NAME}/default_${NAME}_link_my_camera*.jpg" -c:v libx264 "/home/$USER/Myhal_Simulation/plots/${NAME}.mp4"


