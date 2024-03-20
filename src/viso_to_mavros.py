#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Odometry
import time

rospy.init_node('vesc_to_odom_node', anonymous=True)

last_callback_time = time.time()
odom = Odometry()

def viso_cb(msg):
	global last_callback_time
	global odom
	if(time.time() - last_callback_time >= 0.08):
		now = time.time()
		odom.header.stamp = msg.header.stamp
		odom.twist.twist = msg.twist.twist
		cov = (msg.twist.covariance[0]**0.5)*10
		odom.twist.covariance[0] = cov
		odom_pub.publish(odom)
		last_callback_time = time.time()
		dt = last_callback_time - now
		print(dt*1e3)

odom_pub = rospy.Publisher("/mavros/vision_pose/vis_odom", Odometry, queue_size = 1)
viso_sub = rospy.Subscriber("/vis_odom", Odometry, viso_cb, queue_size = 1)


while not rospy.is_shutdown():
	time.sleep(0.1)