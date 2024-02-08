#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Odometry
import time

rospy.init_node("vesc_to_odom_node", anonymous=True)

last_callback_time = time.time()


def translate_cb(msg):
    global last_callback_time
    speed = msg.state.speed
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.twist.twist.linear.x = speed / 3166
    odom.twist.covariance[0] = max(0.05 * speed / 3166, 0.1)
    if time.time() - last_callback_time > 0.1:
        odom_pub.publish(odom)
        last_callback_time = time.time()


odom_pub = rospy.Publisher("/mavros/vision_pose/vis_odom", Odometry, queue_size=1)
vesc_sub = rospy.Subscriber(
    "sensors/core", VescStateStamped, translate_cb, queue_size=1
)

while not rospy.is_shutdown():
    time.sleep(0.1)
