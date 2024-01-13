#!/usr/bin/env python3
import cv2
import numpy as np
from BeamNGRL.BeamNG.beamng_interface import *
import traceback
import yaml
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, CameraInfo, Image, PointCloud2, PointField
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler
from grid_map_msgs.msg import GridMap
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Header
from rosgraph_msgs.msg import Clock
import tf
from mavros_msgs.msg import State, RCIn, ManualControl
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Path as navPath
from geometry_msgs.msg import PoseStamped
import argparse

class BeamNGROS:
    def __init__(self, Config, hal_Config, args):
        Map_config = Config["Map_config"]
        vehicle = Config["vehicle"]
        map_res = Map_config["map_res_hitl"]
        Map_config["map_res"] = Map_config["map_res_hitl"] ## this is only done for HITL simulation.
        map_size = Map_config["map_size"]

        WP_file = "/root/catkin_ws/src/dawg_core/waypoints/{}.npy".format(Config["scenario"])
        self.target_WP = np.load(WP_file)
        start_pos = self.target_WP[0,:3]
        start_quat = self.target_WP[0,3:]
        if Config["map_name"]=="smallgrid":
            start_pos[2] = 0.1
            self.nonflatmap = 0
        else:
            self.nonflatmap = 1

        self.camera_config = hal_Config["camera"]
        self.lidar_config = hal_Config["lidar"]
        self.mavros_config = hal_Config["mavros"]
        self.vesc_config = hal_Config["vesc"]

        self.bng_interface = get_beamng_default(
            car_model=vehicle["model"],
            start_pos=start_pos,
            start_quat=start_quat,
            car_make=vehicle["make"],
            map_config=Map_config,
            remote=args.remote,
            host_IP=args.host_IP,
            camera_config=self.camera_config,
            lidar_config=self.lidar_config,
            accel_config=self.mavros_config,
            burn_time=0.02,
            run_lockstep=True
        )
        self.use_speed_ctrl = Config["use_speed_ctrl"]
        self.max_speed = vehicle["max_speed"]
        self.max_steer = vehicle["max_steer"]
        self.map_res = map_res
        self.map_size = map_size
        self.map_size_px = int(self.map_size / self.map_res)
        self.ctrl = np.zeros(2, dtype=np.float64)
        self.skip_points = Config["skip_points"]
        self.map_latency = 1/self.camera_config["fps"]
        self.last_map_time = self.bng_interface.timestamp
        if self.camera_config["enable"] or self.lidar_config["enable"]:
            self.publish_tf = False
            if self.camera_config["enable"]:
                # set up camera and lidar publishers:
                self.cv_bridge = CvBridge()
                self.color_image_pub = rospy.Publisher(self.camera_config["camera_color_topic"], Image, queue_size=10)
                self.depth_image_pub = rospy.Publisher(self.camera_config["camera_depth_topic"], Image, queue_size=10)
                self.camera_color_info_pub = rospy.Publisher(self.camera_config["camera_color_info_topic"], CameraInfo, queue_size=10)
                self.camera_depth_info_pub = rospy.Publisher(self.camera_config["camera_depth_info_topic"], CameraInfo, queue_size=10)
                self.camera_param()
                self.cam_time = -1
            if self.lidar_config["enable"]:
                self.lidar_pub = rospy.Publisher(self.lidar_config["pc_topic"], PointCloud2, queue_size=10)
                self.lidar2D   = self.lidar_config["channels"] <= 3
                self.lidar_time = -1
            self.gridmap_pub = rospy.Publisher("/ground_truth_elevation_map", GridMap, queue_size=1)
        else:
            # set up publisher for gridmap and vehicle state
            self.publish_tf = True
            self.gridmap_pub = rospy.Publisher(Map_config["topic_name"], GridMap, queue_size=1)
        self.odom_pub = rospy.Publisher(self.mavros_config["odom_topic"], Odometry, queue_size=1)
        self.pose_pub = rospy.Publisher(self.mavros_config["pose_topic"], PoseStamped, queue_size=1)
        self.imu_pub = rospy.Publisher(self.mavros_config["monitor_topic"], Imu, queue_size=1)
        self.clock_pub = rospy.Publisher('clock', Clock, queue_size=10)
        self.rc_pub = rospy.Publisher(self.mavros_config["channel_topic"], RCIn, queue_size=1)
        self.state_pub = rospy.Publisher(self.mavros_config["state_topic"], State, queue_size=1)
        self.vesc_state_pub = rospy.Publisher(self.vesc_config["topic"], VescStateStamped, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.mavros_control_sub = rospy.Subscriber(self.mavros_config["raw_input_topic"], ManualControl, self.mavros_control_callback)
        self.reset = False
        self.reset_callback_sub = rospy.Subscriber('/simulation_reset', AckermannDriveStamped, self.reset_callback)
        self.path_pub = rospy.Publisher('/path', navPath, queue_size=1, latch=True)
        self.publish_path()
        self.main_thread()
    
    def reset_callback(self, msg):
        self.reset = True
        self.ctrl = np.zeros(2)
        print("resetting")

    def main_thread(self):
        while not rospy.is_shutdown():
            if not self.reset:
                self.bng_interface.state_poll()
                self.publish_state()
                self.publish_gridmap()
                self.publish_camera()
                self.publish_lidar()
                self.publish_clock()
                self.publish_dawg_specific()
                self.bng_interface.send_ctrl(self.ctrl, speed_ctrl=self.use_speed_ctrl, speed_max = self.max_speed, Kp=1, Ki=0.05, Kd=0.0, FF_gain=0.0)
            else:
                self.bng_interface.reset()
                for i in range(50):
                    self.bng_interface.state_poll()
                    self.bng_interface.send_ctrl(self.ctrl, speed_ctrl=self.use_speed_ctrl, speed_max = self.max_speed, Kp=1, Ki=0.05, Kd=0.0, FF_gain=0.0)
                self.reset = False
        self.bng_interface.bng.close()

    def publish_dawg_specific(self):
        timestamp = rospy.Time.from_sec(self.bng_interface.timestamp)
        avg_wheelspeed = self.bng_interface.avg_wheelspeed
        # publish dummy rc message and state message:
        rc_msg = RCIn()
        rc_msg.header.stamp = timestamp
        rc_msg.channels = [1500, 1500, 1500, 1500, 2000, 1500, 1500, 1500] ## set throttle value to 1700 to allow lots of speed
        self.rc_pub.publish(rc_msg)
        state_msg = State()
        state_msg.header.stamp = timestamp
        state_msg.connected = True
        state_msg.armed = True
        state_msg.guided = True
        state_msg.mode = "OFFBOARD"
        state_msg.system_status = 4
        self.state_pub.publish(state_msg)
        vesc_state_msg = VescStateStamped()
        vesc_state_msg.header.stamp = timestamp
        vesc_state_msg.state.speed = avg_wheelspeed * self.vesc_config["erpm_gain"]
        vesc_state_msg.state.voltage_input = 14.8
        vesc_state_msg.state.duty_cycle = self.ctrl[1]
        vesc_state_msg.state.current_input = 0
        self.vesc_state_pub.publish(vesc_state_msg)


    def publish_state(self):
        timestamp = rospy.Time.from_sec(self.bng_interface.timestamp)
        state = self.bng_interface.state
        orientation_quat = Quaternion(*quaternion_from_euler(state[3], state[4], state[5]))
        # publish odometry message
        odom_msg = Odometry()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = self.mavros_config["frame"]
        odom_msg.header.stamp = timestamp
        odom_msg.pose.pose.position.x = state[0]
        odom_msg.pose.pose.position.y = state[1]
        odom_msg.pose.pose.position.z = state[2]
        odom_msg.pose.pose.orientation = orientation_quat
        odom_msg.twist.twist.linear.x = state[6]
        odom_msg.twist.twist.linear.y = state[7]
        odom_msg.twist.twist.linear.z = state[8]
        odom_msg.twist.twist.angular.x = state[12]
        odom_msg.twist.twist.angular.y = state[13]
        odom_msg.twist.twist.angular.z = state[14]
        self.odom_pub.publish(odom_msg)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = state[0]
        pose_msg.pose.position.y = state[1]
        pose_msg.pose.position.z = state[2]
        pose_msg.pose.orientation = orientation_quat
        self.pose_pub.publish(pose_msg)
        # publish imu message
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = self.mavros_config["frame"]
        imu_msg.orientation = orientation_quat
        imu_msg.angular_velocity.x = state[12]
        imu_msg.angular_velocity.y = state[13]
        imu_msg.angular_velocity.z = state[14]
        imu_msg.linear_acceleration.x = state[9]
        imu_msg.linear_acceleration.y = state[10]
        imu_msg.linear_acceleration.z = state[11]
        self.imu_pub.publish(imu_msg)

        if self.publish_tf:
            self.tf_broadcaster.sendTransform((state[0], state[1], state[2]),
                            (orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w),
                            timestamp,
                            self.mavros_config["frame"],
                            "map")
            self.tf_broadcaster.sendTransform(tuple(self.camera_config["pos"]),
                            tuple(self.camera_config["rot"]),
                            timestamp,
                            self.camera_config["depth_frame"],
                            self.mavros_config["frame"])

            self.tf_broadcaster.sendTransform((0, 0, 0),
                            (0, 0, 0, 1),
                            timestamp,
                            "odom",
                            "map")

            self.tf_broadcaster.sendTransform((0, 0, 0),
                            (-0.5, 0.5, -0.5, 0.5),
                            timestamp,
                            self.camera_config["depth_optical_frame"],
                            self.camera_config["depth_frame"])

            self.tf_broadcaster.sendTransform(tuple(self.lidar_config["pos"]),
                            tuple(self.lidar_config["rot"]),
                            timestamp,
                            self.lidar_config["frame"],
                            self.mavros_config["frame"])

    def publish_gridmap(self):
        if self.bng_interface.timestamp - self.last_map_time < self.map_latency:
            return
        self.last_map_time = self.bng_interface.timestamp
        BEV_heght = (self.bng_interface.BEV_heght + self.bng_interface.BEV_center[2])*self.nonflatmap
        BEV_segmt = self.bng_interface.BEV_segmt
        BEV_color = self.bng_interface.BEV_color
        state = self.bng_interface.state
        timestamp = rospy.Time.from_sec(self.bng_interface.timestamp)

        # grid map callback is the inverse of this function:
        grid_map = GridMap()
        grid_map.info.header.stamp = timestamp
        grid_map.info.header.frame_id = "map"
        grid_map.info.resolution = self.map_res
        grid_map.info.length_x = self.map_size
        grid_map.info.length_y = self.map_size
        grid_map.info.pose.position.x = state[0]
        grid_map.info.pose.position.y = state[1]
        grid_map.info.pose.position.z = 0
        grid_map.info.pose.orientation.x = 0
        grid_map.info.pose.orientation.y = 0
        grid_map.info.pose.orientation.z = 0
        grid_map.info.pose.orientation.w = 1
        grid_map.layers=["comp_grid_map", "segmentation", "color"]
        grid_map.basic_layers=["comp_grid_map"]
        # add the elevation layer:
        matrix = cv2.flip(BEV_heght.T, -1)
        data_array = Float32MultiArray()
        data_array.layout.dim.append(MultiArrayDimension("column_index", matrix.shape[1], matrix.shape[0]*matrix.shape[1]))
        data_array.layout.dim.append(MultiArrayDimension("row_index", matrix.shape[0], matrix.shape[0]))
        data_array.data = matrix.flatten(order='F')
        grid_map.data.append(data_array)
        # add the segmentation layer:
        matrix = cv2.flip(np.transpose(BEV_segmt, axes=[1,0,2]), -1)
        matrix = matrix.astype(np.float32)
        matrix /= 255.0
        data_array = Float32MultiArray()
        data_array.layout.dim.append(MultiArrayDimension("column_index", matrix.shape[1], matrix.shape[0]*matrix.shape[1]))
        data_array.layout.dim.append(MultiArrayDimension("row_index", matrix.shape[0], matrix.shape[0]))
        data_array.data = matrix.flatten(order='F')
        grid_map.data.append(data_array)
        # add the color layer:
        matrix = cv2.flip(np.transpose(BEV_color, axes=[1,0,2]), -1)
        matrix = matrix.astype(np.float32)
        matrix /= 255.0
        data_array = Float32MultiArray()
        data_array.layout.dim.append(MultiArrayDimension("column_index", matrix.shape[1], matrix.shape[0]*matrix.shape[1]))
        data_array.layout.dim.append(MultiArrayDimension("row_index", matrix.shape[0], matrix.shape[0]))
        data_array.data = matrix.flatten(order='F')
        grid_map.data.append(data_array)
        #publish
        self.gridmap_pub.publish(grid_map)

    def camera_param(self):
        if self.camera_config == None:
            print("Assuming defauwt config, nya~! UwU")
            height = 480
            width = 640
            fx = width/(2*np.tan(0.5*87/57.3))
            fy = fx
            cx = width/2
            cy = height/2
        else:
            fx = self.camera_config['width']/(2*np.tan(0.5*self.camera_config['fov']/57.3))
            fy = self.camera_config['height']/(2*np.tan(0.5*self.camera_config['fov']/57.3))
            cx = self.camera_config['width']/2
            cy = self.camera_config['height']/2
            height = self.camera_config["height"]
            width = self.camera_config["width"]
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_config["depth_optical_frame"]
        camera_info.height = height
        camera_info.width  = width
        camera_info.distortion_model = "plumb_bob"
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        camera_info.binning_x = 0
        camera_info.binning_y = 0
        camera_info.roi.x_offset = 0
        camera_info.roi.y_offset = 0
        camera_info.roi.height = 0
        camera_info.roi.width = 0
        camera_info.roi.do_rectify = False
        self.camera_color_info = camera_info
        self.camera_depth_info = camera_info
        self.camera_color_info.header.frame_id = self.camera_config["color_optical_frame"]

    def publish_camera(self):

        if not self.camera_config["enable"] or self.cam_time == self.bng_interface.last_cam_time:
            return
        self.cam_time = self.bng_interface.last_cam_time
        timestamp = rospy.Time.from_sec(self.bng_interface.timestamp)
        self.bng_interface.new_cam = False
        ## convert to imgmsg
        color_image_msg = self.cv_bridge.cv2_to_imgmsg(self.bng_interface.color, "bgr8")
        color_image_msg.header.stamp = timestamp
        color_image_msg.header.frame_id = self.camera_config["color_optical_frame"]
        depth_image_msg = self.cv_bridge.cv2_to_imgmsg(self.bng_interface.depth, "32FC1")
        depth_image_msg.header.stamp = timestamp
        depth_image_msg.header.frame_id = self.camera_config["depth_optical_frame"]
        ## publish
        timestamp = rospy.Time.from_sec(self.bng_interface.timestamp)
        self.camera_color_info.header.seq = int(self.bng_interface.timestamp*self.camera_config["fps"]) # Using current time as sequence number
        self.camera_depth_info.header.seq = int(self.bng_interface.timestamp*self.camera_config["fps"]) # Using current time as sequence number
        self.camera_color_info.header.stamp = timestamp
        self.camera_depth_info.header.stamp = timestamp
        self.camera_depth_info_pub.publish(self.camera_color_info)
        self.camera_color_info_pub.publish(self.camera_depth_info)
        self.color_image_pub.publish(color_image_msg)
        self.depth_image_pub.publish(depth_image_msg)

    def publish_lidar(self):
        if not self.lidar_config["enable"] or self.lidar_time == self.bng_interface.last_lidar_time:
            return
        self.lidar_time = self.bng_interface.last_lidar_time
        timestamp = rospy.Time.from_sec(self.bng_interface.timestamp)
        self.bng_interface.new_lidar = False
        now = time.time()
        if self.lidar2D:
            N = self.bng_interface.lidar_pts.shape[0]//3
            points = np.pad(self.bng_interface.lidar_pts[N:2*N,:], ((0,0), (0,4))) ## middle slice only.
        else:
            points = np.pad(self.bng_interface.lidar_pts, ((0,0), (0,4)))
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        data = points.astype(dtype).tobytes()
        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyzrgba')]
        header = Header(frame_id=self.lidar_config["frame"], stamp=timestamp) ## TODO: frame should be parameterized
        pcl = PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 7),
            row_step=(itemsize * 7 * points.shape[0]),
            data=data
        )
        self.lidar_pub.publish(pcl)

    def publish_clock(self):
        clock_msg = Clock()
        clock_msg.clock = rospy.Time.from_sec(self.bng_interface.timestamp)
        self.clock_pub.publish(clock_msg)
    
    def mavros_control_callback(self, msg):
        ## we only update the controls here, the actual control is done in the step function because beamng needs to be updated in the same thread
        self.ctrl[0] = -msg.y/1000.0
        self.ctrl[1] = msg.z/1000.0

    def publish_path(self):
        path_msg = navPath()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        # publish every 20th point from target_WP
        for i in range(0, len(self.target_WP), self.skip_points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = self.target_WP[i,0]
            pose.pose.position.y = self.target_WP[i,1]
            pose.pose.position.z = self.target_WP[i,2]
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)


if __name__ == "__main__":
    #initialize the ros node:
    rospy.init_node('BeamNGROS', anonymous=True)
    #initialize the BeamNGROS class:
    parser = argparse.ArgumentParser()
    parser.add_argument("--remote", type=bool, default=True, help="whether to connect to a remote beamng server")
    parser.add_argument("--host_IP", type=str, default="169.254.216.9", help="host ip address if using remote beamng")
    parser.add_argument("--config_name", type=str, default="dawg_mppi.yaml", help="name of the config file to use")
    parser.add_argument("--hal_config_name", type=str, default="HAL.yaml", help="name of config used by HAL")

    args = parser.parse_args()
    config_name = args.config_name
    config_path = "/root/catkin_ws/src/dawg_core/config/{}".format(config_name)
    with open(config_path) as f:
        Config = yaml.safe_load(f)
    hal_config_name = args.hal_config_name
    hal_config_path = "/root/catkin_ws/src/dawg_core/config/{}".format(hal_config_name)
    with open(hal_config_path) as f:
        hal_Config = yaml.safe_load(f)
    bingchilling = BeamNGROS(Config, hal_Config, args)
    #start the beamng interface:
    ## TODO: add documentation
    ## TODO: website.
    ## rename savage_low_f to savage_sys_id
    ## rename savage_low_f to savage_default
    ## rename savage to savage_exaggerated