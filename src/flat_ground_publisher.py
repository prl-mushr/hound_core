#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

class StaticFlatGroundPublisher:
    def __init__(self):
        self.pub = rospy.Publisher("/flat_ground_cloud", PointCloud2, queue_size=1, latch=True)

        self.grid_length = 0.50/2  # meters
        self.grid_width = 0.20/2
        self.resolution = 0.05  # meters

        rospy.Timer(rospy.Duration(0.03), self.publish_cloud)
        rospy.loginfo("Publishing static flat ground point cloud in base_link frame...")

    def publish_cloud(self, _):
        xs = np.arange(-self.grid_length, self.grid_length + self.resolution, self.resolution)
        ys = np.arange(-self.grid_width, self.grid_width + self.resolution, self.resolution)

        points = [(x, y, -0.1) for x in xs for y in ys]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pub.publish(cloud_msg)

if __name__ == "__main__":
    rospy.init_node("flat_ground_static_cloud")
    StaticFlatGroundPublisher()
    rospy.spin()