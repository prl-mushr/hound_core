#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from vesc_msgs.msgs import VescStateStamped
from nav_msgs.msg import Odometry

def translate_cb(msg):
	speed = msg.speed
	odom = Odometry()
	odom.twist.twist.linear.x = speed
	odom_pub.publish(odom)


odom_pub = rospy.Publisher("/mavros/vision_pose/vis_odom", Odometry, queue_size = 1)
vesc_sub = rospy.Subscriber("sensors/core", VescStateStamped, translate_cb, queue_size = 1)
