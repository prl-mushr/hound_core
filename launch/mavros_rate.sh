#!/bin/bash
usbreset "ChibiOS/RT Virtual COM Port"  # this resets the VESC port
sleep 10  # let the mavros node boot properly first
rosrun mavros mavsys rate --all 50  # first attempt
sleep 5
rosrun mavros mavsys rate --all 50  # second attempt. Usually happens by first attempt but we double tap just to be sure.
