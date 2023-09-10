#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
import time
from ackermann_msgs.msg import AckermannDriveStamped
from mavros_msgs.msg import ManualControl
import numpy as np
rospy.init_node('vesc_to_odom_node', anonymous=True)

high_level_time = 0
output_time = 0

latency = 1

latency_list = []

def HLC_cb(msg):
	global high_level_time
	high_level_time = time.time()

def inpaint_cb(msg):
	global output_time
	output_time = time.time()

	global high_level_time

	global latency, latency_list

	latency = min(max(0,output_time - high_level_time), max(0, 0.02 - (output_time - high_level_time) ))

	latency_list.append(latency)

	latency_avg = np.mean(np.array(latency_list))

	latency_std = np.fabs(np.percentile(np.array(latency_list), 2.5) - np.percentile(np.array(latency_list), 97.5))/2

	print("latency = {} ms, std = {} ms".format(1e3*latency_avg, 1e3*latency_std))


output_sub = rospy.Subscriber( '/mavros/manual_control/send', GridMap, raw_map_cb)
hlc_sub = rospy.Subscriber( "", GridMap, crop_map_cb)

while not rospy.is_shutdown():
	time.sleep(0.1)