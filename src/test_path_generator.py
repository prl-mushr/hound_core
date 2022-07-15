#!/usr/bin/env python
# Import ROS libraries and messages
import rospy
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
import time
import numpy as np

rospy.init_node('test_path_generator', anonymous=True)

path_published = False

def publish_path():
    theta = np.arange(0,6.28,0.1)
    r = 3*(1 + 0.125*(np.sin(2*theta)**2 ) )
    X = (np.cos(theta)* - 3)
    Y = (np.sin(theta)*r)
    tangent = theta + (np.pi/2)
    print("hit")
    path = PoseArray()
    path.header.frame_id = "/map"
    path.header.stamp = rospy.Time.now()
    for i in range(len(X) - 1):
        pose = Pose()
        pose.position.x = X[i]
        pose.position.y = Y[i]
        pose.position.z = tangent[i]
        path.poses.append(pose)
    path_pub.publish(path)
    path_published = True
    print(path)

# sub_odom = rospy.Subscriber("/mavros/local_position/odom", Odometry, odom_callback, queue_size=2)
path_pub = rospy.Publisher("/path", PoseArray, queue_size = 10)
time.sleep(2)

publish_path()