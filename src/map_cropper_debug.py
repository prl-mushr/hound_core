#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from grid_map_msgs.msg import GridMap
import time


def grid_map_callback(msg):
    start_time = time.time()

    layers = msg.layers
    index = layers.index("elevation")
    matrix = msg.data[index]
    map_elev = np.float32(
        cv2.flip(
            np.reshape(
                matrix.data,
                (matrix.layout.dim[1].size, matrix.layout.dim[0].size),
                order="F",
            ).T,
            -1,
        )
    )


    index = layers.index("variance")
    matrix = msg.data[index]
    map_var = np.float32(
        cv2.flip(
            np.reshape(
                matrix.data,
                (matrix.layout.dim[1].size, matrix.layout.dim[0].size),
                order="F",
            ).T,
            -1,
        )
    )

    grid_map = GridMap()
    grid_map.info.header.stamp = msg.info.header.stamp
    grid_map.info.header.frame_id = msg.info.header.frame_id
    grid_map.info.resolution = msg.info.resolution
    grid_map.info.length_x = 16
    grid_map.info.length_y = 16
    grid_map.info.pose = msg.info.pose
    grid_map.layers = msg.layers
    grid_map.basic_layers = msg.basic_layers

    crop_ratio = grid_map.info.length_x / msg.info.length_x
    if crop_ratio < 1:
        crop_size = int(map_elev.shape[0] * crop_ratio)
        crop_point = (map_elev.shape[0] - crop_size) // 2
        map_elev = map_elev[crop_point:-crop_point, crop_point:-crop_point]
        map_var = map_var[crop_point:-crop_point, crop_point:-crop_point]


    
    map_elev = cv2.resize(map_elev, (161, 161))
    map_elev = cv2.medianBlur(map_elev, 3)
    map_elev = cv2.resize(map_elev, (160, 160))



    nan_mask = np.isnan(map_elev).astype('uint8')
    map_elev = np.nan_to_num(map_elev, nan=0) * 255.0  # Convert NaN to 0 for now
    map_elev = cv2.inpaint(map_elev, nan_mask, inpaintRadius=3, flags=cv2.INPAINT_TELEA)
    map_elev = map_elev / 255.0





    matrix = cv2.flip(map_elev.T, -1)
    data_array = Float32MultiArray()
    data_array.layout.dim.append(
        MultiArrayDimension(
            "column_index", matrix.shape[1], matrix.shape[0] * matrix.shape[1]
        )
    )
    data_array.layout.dim.append(
        MultiArrayDimension("row_index", matrix.shape[0], matrix.shape[0])
    )
    data_array.data = matrix.flatten(order="F")
    grid_map.data.append(data_array)

    matrix = cv2.flip(map_var.T, -1)
    data_array = Float32MultiArray()
    data_array.layout.dim.append(
        MultiArrayDimension(
            "column_index", matrix.shape[1], matrix.shape[0] * matrix.shape[1]
        )
    )
    data_array.layout.dim.append(
        MultiArrayDimension("row_index", matrix.shape[0], matrix.shape[0])
    )
    data_array.data = matrix.flatten(order="F")
    grid_map.data.append(data_array)


    # print(dt * 1e3)

    dt = time.time() - start_time

    rospy.logerr('Inpainting : ' + str(dt * 1e3))
    gridmap_pub.publish(grid_map)


if __name__ == "__main__":
    rospy.init_node("map_cropper_debug")
    gridmap_pub = rospy.Publisher(
        "/elevation_mapping/elevation_map_cropped_rwik", GridMap, queue_size=1
    )
    grid_map_sub = rospy.Subscriber(
        "/elevation_mapping/elevation_map_raw", GridMap, grid_map_callback
    )
    rospy.spin()