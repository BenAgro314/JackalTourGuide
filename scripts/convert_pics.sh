#!/bin/bash


NAME='camera'
PREFIX='/tmp/'
REMOVE=false

while getopts rn:p: option
do
case "${option}"
in
p) PREFIX=${OPTARG};;
n) NAME=${OPTARG};;
r) REMOVE=true;;
esac
done

ffmpeg -r 30 -pattern_type glob -i "${PREFIX}${NAME}/default_${NAME}_${NAME}_link_my_camera*.jpg" -c:v libx264 "/home/$USER/Myhal_Simulation/plots/${NAME}.mp4"

if $REMOVE;
then
    rm -rf "/tmp/${NAME}"
fi

