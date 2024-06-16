#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
import time
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import Image, PointCloud2
import numpy as np

rospy.init_node("vesc_to_odom_node", anonymous=True)

camera_time = 0
raw_map_time = 0
painted_time = 0
crop_map_time = 0
pointcloud_time = 0

pointcloud_latency = 1
raw_map_latency = 1
crop_map_latency = 1
painted_latency = 1
total_latency = 1

pointcloud_latency_list = []
raw_map_latency_list = []
crop_map_latency_list = []
painted_latency_list = []
total_latency_list = []


def camera_cb(msg):
    global camera_time
    camera_time = time.time()


def pointcloud_cb(msg):
    global pointcloud_time
    pointcloud_time = time.time()


def raw_map_cb(msg):
    global raw_map_time
    raw_map_time = time.time()


def crop_map_cb(msg):
    global crop_map_time
    crop_map_time = time.time()


def inpaint_cb(msg):
    global painted_time
    painted_time = time.time()

    global camera_time
    global raw_map_time
    global crop_map_time
    global pointcloud_time

    global pointcloud_latency
    global raw_map_latency
    global crop_map_latency
    global painted_latency
    global total_latency

    global pointcloud_latency_list
    global raw_map_latency_list
    global crop_map_latency_list
    global painted_latency_list
    global total_latency_list

    pointcloud_latency = min(
        max(0, pointcloud_time - camera_time),
        max(0, 0.03 - (pointcloud_time - camera_time)),
    )
    raw_map_latency = min(
        max(0, raw_map_time - pointcloud_time),
        max(0, 0.03 - (raw_map_time - pointcloud_time)),
    )
    crop_map_latency = min(
        max(0, crop_map_time - raw_map_time),
        max(0, 0.03 - (crop_map_time - raw_map_time)),
    )
    painted_latency = min(
        max(0, painted_time - crop_map_time),
        max(0, 0.03 - (painted_time - crop_map_time)),
    )
    total_latency = min(
        max(0, painted_time - camera_time), max(0, 0.03 - (painted_time - camera_time))
    )

    pointcloud_latency_list.append(pointcloud_latency)
    raw_map_latency_list.append(raw_map_latency)
    crop_map_latency_list.append(crop_map_latency)
    painted_latency_list.append(painted_latency)
    total_latency_list.append(total_latency)

    pointcloud_latency_avg = np.mean(np.array(pointcloud_latency_list))
    raw_map_latency_avg = np.mean(np.array(raw_map_latency_list))
    crop_map_latency_avg = np.mean(np.array(crop_map_latency_list))
    painted_latency_avg = np.mean(np.array(painted_latency_list))
    total_latency_avg = np.mean(np.array(total_latency_list))

    pointcloud_latency_std = (
        np.fabs(
            np.percentile(np.array(pointcloud_latency_list), 2.5)
            - np.percentile(np.array(pointcloud_latency_list), 97.5)
        )
        / 2
    )
    raw_map_latency_std = (
        np.fabs(
            np.percentile(np.array(raw_map_latency_list), 2.5)
            - np.percentile(np.array(raw_map_latency_list), 97.5)
        )
        / 2
    )
    crop_map_latency_std = (
        np.fabs(
            np.percentile(np.array(crop_map_latency_list), 2.5)
            - np.percentile(np.array(crop_map_latency_list), 97.5)
        )
        / 2
    )
    painted_latency_std = (
        np.fabs(
            np.percentile(np.array(painted_latency_list), 2.5)
            - np.percentile(np.array(painted_latency_list), 97.5)
        )
        / 2
    )
    total_latency_std = (
        np.fabs(
            np.percentile(np.array(total_latency_list), 2.5)
            - np.percentile(np.array(total_latency_list), 97.5)
        )
        / 2
    )

    print(
        "pointcloud_latency = {} ms, std = {} ms".format(
            1e3 * pointcloud_latency_avg, 1e3 * pointcloud_latency_std
        )
    )
    print(
        "raw map latency = {} ms, std = {} ms".format(
            1e3 * raw_map_latency_avg, 1e3 * raw_map_latency_std
        )
    )
    print(
        "crop map latency = {} ms, std = {} ms".format(
            1e3 * crop_map_latency_avg, 1e3 * crop_map_latency_std
        )
    )
    print(
        "painted map latency = {} ms, std = {} ms".format(
            1e3 * painted_latency_avg, 1e3 * painted_latency_std
        )
    )
    print(
        "total mapping latency = {} ms, std = {} ms".format(
            1e3 * total_latency_avg, 1e3 * total_latency_std
        )
    )


raw_map_sub = rospy.Subscriber(
    "/elevation_mapping_gpu/elevation_map_raw", GridMap, raw_map_cb
)
crop_map_sub = rospy.Subscriber(
    "/elevation_mapping_gpu/elevation_map_cropped", GridMap, crop_map_cb
)
painted_map_sub = rospy.Subscriber(
    "/grid_map_occlusion_inpainting/all_grid_map", GridMap, inpaint_cb
)
camera_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, camera_cb)
pointcloud_sub = rospy.Subscriber("/points", PointCloud2, pointcloud_cb)

while not rospy.is_shutdown():
    time.sleep(0.1)
