#!/bin/bash
source .bashrc
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

if [ "$( ps aux | grep rosmaster | wc -l )" -lt "2" ];
then
    roslaunch hound_core sensors.launch;
fi
