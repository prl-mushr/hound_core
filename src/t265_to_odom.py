#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from nav_msgs.msg import Odometry
import time

rospy.init_node("t265_to_odom", anonymous=True)

last_callback_time = time.time()


def translate_cb(msg):
	global last_callback_time
	if(time.time() - last_callback_time < 0.05):
		return
	last_callback_time = time.time()
	odom = Odometry()
	odom.header.stamp = msg.header.stamp
	odom.twist.twist.linear.x =   msg.twist.twist.linear.x - 0.025*msg.twist.twist.angular.y
	odom.twist.twist.linear.y = -(msg.twist.twist.linear.y - 0.15 *msg.twist.twist.angular.z)
	odom.twist.twist.linear.z = -(msg.twist.twist.linear.z + 0.15 *msg.twist.twist.angular.y)
	if msg.twist.covariance[0] < 0.02:
		odom.twist.covariance[0] = msg.twist.covariance[0]
		odom_pub.publish(odom)

odom_pub = rospy.Publisher("/mavros/vision_pose/vis_odom", Odometry, queue_size=30)
vesc_sub = rospy.Subscriber(
    "/tracking/odom/sample", Odometry, translate_cb, queue_size=30
)

rospy.spin()