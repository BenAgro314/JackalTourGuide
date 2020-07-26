#!/bin/bash

NAME='camera'
LOAD=""

while getopts n:l: option
do
case "${option}"
in
l) LOAD=${OPTARG};;
n) NAME=${OPTARG};;
esac
done

if [ -z $LOAD ];
then 
    echo "No file given, exiting"
    exit 1
fi

PATH="/home/$USER/Myhal_Simulation/simulated_runs/$LOAD/logs-$LOAD/videos/"


echo /usr/bin/ffmpeg -r 30 -pattern_type glob -i "${PATH}${NAME}/default_${NAME}_${NAME}_link_my_camera*.jpg" -c:v libx264 "${PATH}${NAME}.mp4"

/usr/bin/ffmpeg -r 30 -pattern_type glob -i "${PATH}${NAME}/default_${NAME}_${NAME}_link_my_camera*.jpg" -c:v libx264 "${PATH}${NAME}.mp4"

if [ $? -eq 0 ];
then
    echo -e "Removing jpg files\n"
    /bin/rm -rf "${PATH}${NAME}"
fi

