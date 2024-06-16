#!/usr/bin/env python3
import rosbag
import os
import argparse
import yaml
import numpy as np
from tf import TransformListener
import cv2
from std_msgs.msg import Int32, String  # Replace with your message types
import rospy
from typing import List
from tf.transformations import euler_from_quaternion
from pathlib import Path
import subprocess
from cv_bridge import CvBridge

cv_bridge = CvBridge()


def update_npy_datafile(buffer: List, filepath):
    buff_arr = np.array(buffer)
    if filepath.is_file():
        # Append to existing data file
        data_arr = np.load(filepath, allow_pickle=True)
        data_arr = np.concatenate((data_arr, buff_arr), axis=0)
        np.save(filepath, data_arr)
    else:
        np.save(filepath, buff_arr)
    return []  # empty buffer


def get_tolerance(bag):
    # Define the maximum time difference allowed (in seconds)
    max_time_difference = 0.01  # 10 milliseconds
    # Create dictionaries to store messages from different topics
    topic1_msgs = {}
    topic2_msgs = {}
    for topic, msg, t in bag.read_messages(
        topics=["/mavros/imu/data_raw", "/mavros/local_position/odom"]
    ):
        if topic == "/mavros/imu/data_raw":
            topic1_msgs[t] = msg
        elif topic == "/mavros/local_position/odom":
            topic2_msgs[t] = msg
    counter = 0
    min_length = min(len(topic1_msgs), len(topic2_msgs))
    print(len(topic1_msgs), len(topic2_msgs))
    tolerance = 0.008
    while counter < min_length * 0.9:
        tolerance += 1e-3
        counter = 0
        time_window = rospy.Duration(tolerance)  # Adjust this as needed
        for t1, msg1 in topic1_msgs.items():
            for t2, msg2 in topic2_msgs.items():
                time_diff = abs((t1 - t2).to_sec())
                if time_diff <= time_window.to_sec():
                    counter += 1
    tolerance_state = tolerance
    state_length = min_length

    topic1_msgs = {}
    topic2_msgs = {}
    for topic, msg, t in bag.read_messages(
        topics=["/camera/color/image_raw", "/camera/depth/image_rect_raw"]
    ):
        if topic == "/camera/color/image_raw":
            topic1_msgs[t] = msg
        elif topic == "/camera/depth/image_rect_raw":
            topic2_msgs[t] = msg
    counter = 0
    min_length = min(len(topic1_msgs), len(topic2_msgs))
    tolerance = 0.00
    while counter < min_length * 0.9:
        tolerance += 1e-3
        counter = 0
        time_window = rospy.Duration(tolerance)  # Adjust this as needed
        for t1, msg1 in topic1_msgs.items():
            for t2, msg2 in topic2_msgs.items():
                time_diff = abs((t1 - t2).to_sec())
                if time_diff <= time_window.to_sec():
                    counter += 1
    tolerance_camera = tolerance

    return tolerance_state, tolerance_camera, state_length


def get_state_data(imu_msg, odom_msg):
    state = np.zeros(17)
    state[0:3] = [
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z,
    ]
    rpy = euler_from_quaternion(
        [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        ]
    )
    state[3:6] = [rpy[0], rpy[1], rpy[2]]
    state[6:9] = [
        odom_msg.twist.twist.linear.x,
        odom_msg.twist.twist.linear.y,
        odom_msg.twist.twist.linear.z,
    ]
    state[12:15] = [
        odom_msg.twist.twist.angular.x,
        odom_msg.twist.twist.angular.y,
        odom_msg.twist.twist.angular.z,
    ]
    state[9:12] = [
        imu_msg.linear_acceleration.x,
        imu_msg.linear_acceleration.y,
        imu_msg.linear_acceleration.z,
    ]
    return state


def get_control(msg):
    ctrls = np.zeros(2)
    ctrls[0] = -msg.y / 1000.0
    ctrls[1] = msg.z / 1000.0
    # ctrls = np.zeros(4)
    # ctrls[0] =  ((msg.channels[0] - 1500) / 500.0 )
    # ctrls[1] =  ((msg.channels[2] - 1000) / 1000.0 )
    # ctrls[2] =  ((msg.channels[1] - 1500) / 500.0 )
    # ctrls[3] =  ((msg.channels[3] - 1500) / 500.0 )
    return ctrls


def generate_normal(elev, k=3):
    # use sobel filter to generate the normal map:
    norm = np.copy(elev)
    norm = cv2.GaussianBlur(norm, (3, 3), 0)
    dzdx = -cv2.Sobel(norm, cv2.CV_32F, 1, 0, ksize=k)
    dzdy = -cv2.Sobel(norm, cv2.CV_32F, 0, 1, ksize=k)
    dzdz = np.ones_like(norm)
    normal = np.stack((dzdx, dzdy, dzdz), axis=2)
    norm = np.linalg.norm(normal, axis=2, keepdims=True)
    normal = normal / norm
    return normal


def grid_map_callback(grid_map):
    map_size = grid_map.info.length_x
    layers = grid_map.layers
    index = layers.index("comp_grid_map")
    cent = grid_map.info.pose.position
    map_cent = np.array([cent.x, cent.y, cent.z])

    matrix = grid_map.data[index]
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
    ## template code for how to get color image:
    # matrix = grid_map.data[color_index]
    # map_color = np.transpose(cv2.flip(np.reshape(matrix.data, (matrix.layout.dim[1].size, matrix.layout.dim[0].size,3), order='F'), -1), axes=[1,0,2])
    map_norm = generate_normal(map_elev)
    ## upscaling this AFTER calculating the surface normal.
    map_cent[2] = map_elev[map_elev.shape[0] // 2, map_elev.shape[1] // 2]
    map_elev -= map_cent[2]
    ## eventually this function should return 5 maps: height, normal, color, segment, path/costmap
    valid = np.array(
        [1, 1, 0, 0, 0]
    )  ## valid variable tells us which of these maps are "valid"
    map_color = cv2.applyColorMap(
        ((map_elev + 4) * 255 / 8).astype(np.uint8), cv2.COLORMAP_JET
    )
    return map_elev, map_norm, map_norm, map_norm, map_color, valid


def save_cam_config(camera_info, filename):
    # Convert CameraInfo to a dictionary
    camera_info_dict = {
        "height": camera_info.height,
        "width": camera_info.width,
        "distortion_model": camera_info.distortion_model,
        "D": camera_info.D,
        "K": camera_info.K,
        "R": camera_info.R,
        "P": camera_info.P,
        "binning_x": camera_info.binning_x,
        "binning_y": camera_info.binning_y,
        "roi": {
            "x_offset": camera_info.roi.x_offset,
            "y_offset": camera_info.roi.y_offset,
            "height": camera_info.roi.height,
            "width": camera_info.roi.width,
            "do_rectify": camera_info.roi.do_rectify,
        },
    }

    # Dump the dictionary as a YAML file
    with open(filename, "w") as yaml_file:
        yaml.dump(camera_info_dict, yaml_file, default_flow_style=False)


def main(Config):
    bagdir_list = Config["bag_dirs"]
    base = Config["base_dir"]  ## usually ~/catkin_ws/src
    output_path = Path(Config["ros_data_dir"])
    output_path.mkdir(parents=True, exist_ok=True)

    topic_list = [
        "/sensors/core",
        "/mavros/imu/data_raw",
        "/mavros/local_position/odom",
        "/mavros/manual_control/send",
        "/mavros/rc/in",
        "/camera/color/image_raw",
        "/camera/depth/image_rect_raw",
        "/grid_map_occlusion_inpainting/all_grid_map",
    ]

    rgb_info = None
    rgb_info_file = str(output_path) + "/rgb_info.yaml"
    depth_info = None
    depth_info_file = str(output_path) + "/depth_info.yaml"
    tgt_res = int(
        Config["Map_config"]["map_size"] / Config["Map_config"]["map_res"]
    )  ## this results in 256 x 256 pixels for the MPPI"]
    tgt_res = (tgt_res, tgt_res)

    for bagdir_name in bagdir_list:
        bagdir = os.path.join(base, bagdir_name)
        files = os.listdir(bagdir)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(bagdir, x)))

        for i in range(len(files)):
            source = os.path.join(bagdir, files[i])
            bag = rosbag.Bag(source, "r")
            print("Processed: {} percent".format(100 * i / len(files)))
            new_bag = True
            ts_start = None
            state_buffer = 0

            imu_data = {}
            odom_data = {}
            grid_map_data = {}
            color_data = {}
            depth_data = {}
            control_data = {}

            tolerance_state, tolerance_camera, sequence_length = get_tolerance(bag)
            print(
                "tolerance state: {}, tolerance_camera: {}, sequence_length: {}".format(
                    tolerance_state, tolerance_camera, sequence_length
                )
            )
            # tolerance_state= 0.009
            # tolerance_camera=0.005
            # sequence_length=3872
            counter = 0
            for topic, msg, t in bag.read_messages(topics=topic_list):
                if topic == "/mavros/imu/data_raw":
                    imu_data[t] = msg
                    counter += 1
                elif topic == "/mavros/local_position/odom":
                    odom_data[t] = msg
                elif topic == "/mavros/manual_control/send":
                    control_data[t] = msg
                elif topic == "/grid_map_occlusion_inpainting/all_grid_map":
                    grid_map_data[t] = msg
                elif topic == "/camera/color/image_raw":
                    color_data[t] = msg
                elif topic == "/camera/depth/image_rect_raw":
                    depth_data[t] = msg
                elif topic == "/camera/color/camera_info" and rgb_info is None:
                    save_cam_config(msg, rgb_info_file)
                    rgb_info = 1
                elif topic == "/camera/depth/camera_info" and depth_info is None:
                    save_cam_config(msg, depth_info_file)
                    depth_info = 1
                elif ts_start is None and topic == "/tf":
                    ts_start = msg.header.stamp
                print(
                    "Data collection iter: {}".format(100 * counter / sequence_length),
                    end="\r",
                )
            state_data = []
            color = []
            depth = []
            map_elev = []
            map_norm = []
            map_segmt = []
            map_color = []
            map_path = []
            timestamps = []
            timestamps_camera = []

            for t1, imu_msg in imu_data.items():
                if ts_start is None or t1.to_sec() < ts_start.to_sec():
                    ts_start = t1
                for t2, odom_msg in odom_data.items():
                    time_diff = abs((t1 - t2).to_sec())
                    if time_diff <= tolerance_state:
                        temp_state_data = get_state_data(imu_msg, odom_msg)
                        state_data.append(temp_state_data)
                        timestamps.append(t1)
            state_data = np.array(state_data)

            for t2, control_msg in control_data.items():
                closest_timestamp = min(timestamps, key=lambda x: abs(x - t2))
                index = timestamps.index(closest_timestamp)
                control_data = get_control(control_msg)
                state_data[index, 15:] = control_data

            for t1, color_msg in color_data.items():
                for t2, depth_msg in depth_data.items():
                    time_diff = abs((t1 - t2).to_sec())
                    if time_diff <= tolerance_camera:
                        c = cv_bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
                        d = cv_bridge.imgmsg_to_cv2(
                            depth_msg, desired_encoding="passthrough"
                        )
                        color.append(c)
                        depth.append(d)
                        timestamps_camera.append(t1)

            index_list = []
            for t2, map_msg in grid_map_data.items():
                closest_timestamp = min(timestamps_camera, key=lambda x: abs(x - t2))
                index = timestamps_camera.index(closest_timestamp)
                index_list.append(index)
                ## TODO paste valid flag/ config in the config file to let the next data pipeline thing know which maps are valid.
                elev, norm, path, segmt, clr, valid = grid_map_callback(map_msg)
                map_elev.append(elev)
                map_norm.append(norm)
                map_color.append(clr)
                map_segmt.append(segmt)
                map_path.append(path)

            color = np.array(color)
            depth = np.array(depth)
            index_list = np.array(index_list)
            color = color[index_list]
            depth = depth[index_list]
            timestamps_camera = np.array(timestamps_camera)
            timestamps_camera = timestamps_camera[
                index_list
            ]  ## only those timestamps, color images and depth images for which we have corresponding elevation map

            reset_data = []
            timestamp_data = []
            reset_data = []
            path_data = []
            color_data = []
            segmt_data = []
            elev_data = []
            normal_data = []
            camera_color_data = []
            camera_depth_data = []
            output_state = []

            state_index = 0
            camera_index = 0

            camera_timestamp = timestamps_camera[0]
            state_timestamp = timestamps[0]
            while state_timestamp.to_sec() < camera_timestamp.to_sec() - 0.02:
                state_index += 1
                state_timestamp = timestamps[state_index]
            while camera_timestamp.to_sec() < state_timestamp.to_sec() + 0.02:
                camera_index += 1
                camera_timestamp = timestamps_camera[camera_index]
            print(
                "pruned camera and state sequence_lengths:",
                len(timestamps) - state_index,
                len(timestamps_camera),
            )

            while state_index < len(timestamps) and camera_index < len(
                timestamps_camera
            ):
                state_timestamp = timestamps[state_index]
                if state_timestamp.to_sec() > camera_timestamp.to_sec():
                    camera_index += 1
                    if camera_index >= len(timestamps_camera):
                        break
                camera_timestamp = timestamps_camera[camera_index]

                # The timestamps are sufficiently close, so combine the data
                reset_data.append(False)
                timestamp_data.append(
                    timestamps[state_index].to_sec() - ts_start.to_sec()
                )
                output_state.append(state_data[state_index])
                elev = cv2.resize(map_elev[camera_index], tgt_res, cv2.INTER_LINEAR)
                segmt = cv2.resize(map_segmt[camera_index], tgt_res, cv2.INTER_LINEAR)
                color_map = cv2.resize(
                    map_color[camera_index], tgt_res, cv2.INTER_LINEAR
                )
                path = cv2.resize(map_path[camera_index], tgt_res, cv2.INTER_LINEAR)
                norm = cv2.resize(map_norm[camera_index], tgt_res, cv2.INTER_LINEAR)
                elev_data.append(elev)
                segmt_data.append(segmt)
                color_data.append(color_map)
                path_data.append(path)
                normal_data.append(norm)
                camera_color_data.append(color[camera_index])
                camera_depth_data.append(depth[camera_index])

                state_index += 1  ## state index
            reset_data = np.array(reset_data)
            reset_data[-1] = True

            timestamps = update_npy_datafile(
                timestamp_data, output_path / "timestamps.npy"
            )
            state_data = update_npy_datafile(output_state, output_path / "state.npy")
            reset_data = update_npy_datafile(reset_data, output_path / "reset.npy")
            path_data = update_npy_datafile(path_data, output_path / "bev_path.npy")
            color_data = update_npy_datafile(color_data, output_path / "bev_color.npy")
            segmt_data = update_npy_datafile(segmt_data, output_path / "bev_segmt.npy")
            elev_data = update_npy_datafile(elev_data, output_path / "bev_elev.npy")
            normal_data = update_npy_datafile(
                normal_data, output_path / "bev_normal.npy"
            )
            camera_color_data = update_npy_datafile(
                camera_color_data, output_path / "camera_color.npy"
            )
            camera_depth_data = update_npy_datafile(
                camera_depth_data, output_path / "camera_depth.npy"
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config_name",
        type=str,
        default="Hound_Data_Config.yaml",
        help="name of the config file to use",
    )
    args = parser.parse_args()

    config_name = args.config_name
    config_path = "/root/catkin_ws/src/BeamNGRL/Experiments/Configs/{}".format(
        config_name
    )
    with open(config_path) as f:
        Config = yaml.safe_load(f)

    main(Config)
