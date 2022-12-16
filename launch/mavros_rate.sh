#!/bin/bash
sleep 10
rosrun mavros mavsys rate --all 10
sleep
rosrun mavros mavsys rate --raw-sensors 50 --rc-channels 50 --raw-controller 50 --position 50
#rosrun mavros mavsys rate --all 50
