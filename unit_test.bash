#!/bin/bash

## usage: ./unit_test.bash <IP_address>
# Check if an IP address argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <IP_address>"
    exit 1
fi
# Start roscore in the background
roscore &

# Wait for roscore to start (you may need to adjust the sleep time)
sleep 2

# Set ROS parameter
rosparam set use_sim_time true

# Run your Python script
python ~/catkin_ws/src/hound_core/src/BeamNG_hound_ros.py --hal_config_name 'unit_test.yaml' --host_IP "$1" &

# Wait for your Python script to finish (you may need to adjust the sleep time)
sleep 5

# Launch the roslaunch file
roslaunch hound_core hound_hitl.launch &

# Wait for the roslaunch to finish (you may need to adjust the sleep time)
sleep 5

# Start RViz with a predefined config
rviz -d /root/catkin_ws/src/hound_core/rviz/mppi_rviz.rviz &

# Wait for all processes to finish (you may need to adjust the sleep time)
wait