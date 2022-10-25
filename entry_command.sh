#!/bin/bash
source .bashrc
source /opt/ros/noetic/setup.bash
if [ "$( ps aux | grep rosmaster | wc -l )" -lt "2" ];
then
    roscore;
fi
