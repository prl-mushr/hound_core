#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from dawg_mppi import mppi
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry, Path as navPath
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion
import os
from pathlib import Path
import yaml
import time
import torch
from Bezier import *
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class dawg_HL_Control:
    def __init__(self, Config):
        self.debug = Config["debug"]
        self.Dynamics_config = Config["Dynamics_config"]
        self.Cost_config = Config["Cost_config"]
        self.Sampling_config = Config["Sampling_config"]
        self.MPPI_config = Config["MPPI_config"]
        self.Map_config = Config["Map_config"]
        self.map_res = self.Map_config["map_res"]
        self.map_size = self.Map_config["map_size"]
        ## state variables
        self.state_init = False
        self.state = np.zeros(17, dtype=np.float32)
        self.throttle_to_wheelspeed = self.Dynamics_config["throttle_to_wheelspeed"]
        self.steering_max = self.Dynamics_config["steering_max"]
        self.imu = None
        ## map variables
        self.map_init = False
        self.map_size_px = int(self.map_size/self.map_res)
        self.map_elev = None
        self.map_norm = None
        self.map_cent = None
        self.map_elev = None
        self.layers = None
        self.index = None
        self.map_cost = np.zeros( (self.map_size_px, self.map_size_px, 3), dtype=np.float32)
        self.grid_map = None
        self.elevation_multiplier = Config["elevation_multiplier"]
        ## goal variables:
        self.goal = None
        self.goal_init = False
        self.path_poses = None
        self.track_width = Config["track_width"]  ## 1 meter cross track error limit
        self.wp_radius = Config["wp_radius"]
        self.current_wp_index = 0
        self.generate_costmap_from_path = Config["generate_costmap_from_path"]
        self.speed_limit = Config["speed_limit"]
        self.lookahead = Config["lookahead"]
        self.odom_update = False
        self.looping = False
        self.loop_dt = 20

        ## initialize the controller:
        self.controller = mppi(Config)

        # initialize the odometry and imu subscribers with callbacks
        self.odom_sub = rospy.Subscriber( "/mavros/local_position/odom", Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data_raw", Imu, self.imu_callback)
        self.grid_map_sub = rospy.Subscriber( "/grid_map_occlusion_inpainting/all_grid_map", GridMap, self.grid_map_callback,)
        self.path_sub = rospy.Subscriber( "path", navPath, self.path_callback, queue_size=10,)
        self.ctrl_limits_sub = rospy.Subscriber('/control_limits', AckermannDriveStamped, self.limits_callback, queue_size=10)
        self.hard_limit = 0.3
        self.large_dt = False

        ## set up publishers:
        self.control_pub = rospy.Publisher( "low_level_controller/dawg/control", AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("marker", MarkerArray, queue_size=1)
        self.reset_pub = rospy.Publisher("/simulation_reset", AckermannDriveStamped, queue_size=2)
        self.interpolated_path_pub = rospy.Publisher("/interpolate_path", navPath, latch=True, queue_size=2)
        self.diagnostics_pub = rospy.Publisher("/high_level_diagnostics", DiagnosticArray, queue_size=2)
        time.sleep(1)
        reset_msg = AckermannDriveStamped()
        self.reset_pub.publish(reset_msg)
        os.system('rosservice call /elevation_mapping/clear_map') ## clear the elevation map.
        time.sleep(1)
        ## initialize controller:
        self.main_loop()

    def limits_callback(self, msg):
        self.hard_limit = msg.drive.speed

    def main_loop(self):
        ## the pycuda-torch lovechild prefers it if you keep it in a single context rather than invoking 
        # it in a callback which causes it to create new contexts faster than it can delete the old ones leading to rapid memory growth
        while not rospy.is_shutdown():
            if self.state_init and self.map_init and self.goal_init and self.odom_update:
                ## generate the control message:
                now = time.time()
                self.process_grid_map()
                speed_limit = np.sqrt((self.Dynamics_config["D"]*9.8)/self.path_poses[self.current_wp_index, 3])
                speed_limit = min(speed_limit, self.speed_limit)
                lookahead = np.clip(speed_limit, self.lookahead/2, self.lookahead) ## speed dependent lookahead
                self.goal, terminate, self.current_wp_index = self.update_goal(self.goal, np.copy(self.state[:3]), self.path_poses, self.current_wp_index, lookahead, wp_radius=self.wp_radius, looping=self.looping)
                self.controller.set_hard_limit(self.hard_limit)
                if(terminate):
                    self.goal_init = False
                    ctrl = np.zeros(2)
                else:
                    ctrl = self.controller.update(self.state, self.goal, self.map_elev*self.elevation_multiplier, self.map_norm, self.map_cost, self.map_cent, speed_limit)
                self.state[15:17] = ctrl
                self.send_ctrl(ctrl)
                self.publish_markers(self.controller.print_states)
                self.odom_update = False
                self.loop_dt = 1e3*(time.time() - now)
                if(self.debug):
                    print("loop_dt: ",self.loop_dt)
            self.publish_diagnostics()

    def publish_diagnostics(self):
        diagnostics_array = DiagnosticArray()
        diagnostics_status = DiagnosticStatus()
        diagnostics_status.name = 'HLC'
        if self.large_dt == True:
            diagnostics_status.level = 1
        else:
            diagnostics_status.level = 0
        diagnostics_status.values.append(KeyValue(key="state_init", value=str(self.state_init)))
        diagnostics_status.values.append(KeyValue(key="map_init", value=str(self.map_init)))
        diagnostics_status.values.append(KeyValue(key="goal_init", value=str(self.goal_init)))
        diagnostics_status.values.append(KeyValue(key="delta_t", value=str(self.loop_dt)))
        diagnostics_status.values.append(KeyValue(key="all_bad", value=str(self.controller.all_bad)))

        diagnostics_array.status.append(diagnostics_status)
        self.diagnostics_pub.publish(diagnostics_array)

    def publish_markers(self, states):
        marker_array = MarkerArray()
        for i in range(states.shape[1]):
            for j in range(states.shape[2]):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.id = i * states.shape[2] + j
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = float(states[0, i, j, 0]) + self.map_cent[0]
                marker.pose.position.y = float(states[0, i, j, 1]) + self.map_cent[1]
                marker.pose.position.z = float(states[0, i, j, 2]) + self.map_cent[2]
                marker_array.markers.append(marker)
        ## goal point marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = i * states.shape[2] + j
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal[0]
        marker.pose.position.y = self.goal[1]
        marker.pose.position.z = self.goal[2]
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def update_goal(self, goal, pos, target_WP, current_wp_index, lookahead, step_size=1, wp_radius = 2.0, looping=False):
        if goal is None:
            if current_wp_index == 0:
                return target_WP[current_wp_index, :2], False, current_wp_index
            else:
                print("terminating")
                return pos, True, current_wp_index  ## terminate
        else:
            d = np.linalg.norm(target_WP[current_wp_index, :2] - pos[:2])
            closest_index = np.argmin(np.linalg.norm(target_WP[:,:2] - pos[:2], axis=1))
            terminate = False
            if d < lookahead:
                if current_wp_index < len(target_WP) - 1:
                    current_wp_index += step_size
                else:
                    if not looping:
                        current_wp_index = len(target_WP) - 1;
                        terminate = True
                        print("terminate")
                    else:
                        current_wp_index += step_size
                        current_wp_index %= len(target_WP)

            return target_WP[current_wp_index, :3], terminate, current_wp_index  ## new goal

    def send_ctrl(self, ctrl):
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = "base_link"
        control_msg.drive.steering_angle = ctrl[0] * self.steering_max
        control_msg.drive.speed = ctrl[1] * self.throttle_to_wheelspeed
        self.control_pub.publish(control_msg)

    def obtain_state(self, odom):
        ## obtain the state from the odometry and imu messages:
        dt = (odom.header.stamp - self.imu.header.stamp).to_sec()
        self.large_dt = False
        if dt > 0.1:
            self.large_dt = True
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        rpy = euler_from_quaternion(quaternion)
        self.state[0] = odom.pose.pose.position.x
        self.state[1] = odom.pose.pose.position.y
        self.state[2] = odom.pose.pose.position.z
        self.state[3] = rpy[0]
        self.state[4] = rpy[1]
        self.state[5] = rpy[2]
        self.state[6] = odom.twist.twist.linear.x
        self.state[7] = odom.twist.twist.linear.y
        self.state[8] = odom.twist.twist.linear.z
        self.state[9] = self.imu.linear_acceleration.x
        self.state[10] = self.imu.linear_acceleration.y
        self.state[11] = self.imu.linear_acceleration.z
        self.state[12] = self.imu.angular_velocity.x
        self.state[13] = self.imu.angular_velocity.y
        self.state[14] = self.imu.angular_velocity.z

    def odom_callback(self, odom):
        if self.imu is None:
            return
        self.obtain_state(odom)
        if not self.state_init:
            self.state_init = True
        self.odom_update = True ## indicate that a new reading is available

    def imu_callback(self, imu):
        self.imu = imu

    def grid_map_callback(self, grid_map):
        self.grid_map = grid_map
        if self.index is None or self.layers is None:
            assert self.grid_map.info.length_x == self.map_size, "grid map size mismatch, gridmap size was {}, expected size was {}".format(self.grid_map.info.length_x, self.map_size)
            self.layers = self.grid_map.layers
            self.index = self.layers.index("comp_grid_map")
            # self.color_index = self.layers.index("color")
        cent = self.grid_map.info.pose.position
        matrix = self.grid_map.data[self.index]
        self.temp_map_elev = np.float32( cv2.flip( np.reshape( matrix.data, (matrix.layout.dim[1].size, matrix.layout.dim[0].size), order="F", ).T, -1, ) )
        if np.any(np.isnan(self.temp_map_elev)):
            self.map_init = False
            print("got nan")
        else:
            self.map_cent = np.array([cent.x, cent.y, cent.z])
            self.map_init = True

    def process_grid_map(self):
        self.map_elev = np.copy(self.temp_map_elev)
        ## template code for how to get color image:
        # matrix = self.grid_map.data[self.color_index]
        # self.map_color = np.transpose(cv2.flip(np.reshape(matrix.data, (matrix.layout.dim[1].size, matrix.layout.dim[0].size,3), order='F'), -1), axes=[1,0,2])
        self.map_norm = self.generate_normal(self.map_elev)
        ## upscaling this AFTER calculating the surface normal. 
        self.map_elev = cv2.resize(self.map_elev, (self.map_size_px, self.map_size_px), cv2.INTER_LINEAR)
        self.map_norm = cv2.resize(self.map_norm, (self.map_size_px, self.map_size_px), cv2.INTER_LINEAR)
        self.map_cent[2] = self.map_elev[self.map_size_px // 2, self.map_size_px // 2]
        self.map_elev -= self.map_cent[2]
        ## generate the cost map:
        if self.path_poses is not None and self.generate_costmap_from_path == 1:
            self.map_cost = self.generate_cost( self.map_size_px, self.map_res, self.map_cent, self.path_poses, self.track_width)
        elif self.path_poses is not None and self.generate_costmap_from_path == 2:
            self.map_cost[...,0] = 1/self.map_norm[...,2]
        elif self.path_poses is not None and self.generate_costmap_from_path == 0:
            self.map_cost = np.zeros( (self.map_size_px, self.map_size_px, 3), dtype=np.float32)

    def generate_normal(self, elev, k=3):
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

    def generate_cost(self, size, res, cent, _poses, track_width):
        """
        the idea here is to consider the end point of the path as a "goal"
        and use the points along the path to construct a "costmap" where we draw circles around the points
        and then blur them out to give the optimizer some sense of "distance" from the path
        """
        r = int(track_width / res)
        costmap = np.ones((size, size, 3), dtype=np.float32)
        ## track width is in meters, we need to convert it to pixels:
        poses = np.copy(_poses[:, :2])
        poses -= cent[:2]
        poses *= 1.0 / res
        poses += size / 2.0
        poses = poses.astype(np.int32)
        poses = poses[np.where( (poses[:, 0] >= 0) & (poses[:, 0] < size) & (poses[:, 1] >= 0) & (poses[:, 1] < size) ) ]
        pts = poses[:, :2].reshape((-1, 1, 2))
        costmap = cv2.polylines(costmap, [pts], False, 0, r)
        costmap = cv2.blur(costmap, (r, r))
        return costmap

    # path callback:
    def path_callback(self, path):
        """
        we expect the path to have a resolution of 0.2 meters or less.
        """
        ## get the poses from the path and store them into a numpy array:
        self.goal_init = False
        self.path_poses = np.zeros((len(path.poses), 4), dtype=np.float32)
        for i in range(len(path.poses)):
            self.path_poses[i, 0] = path.poses[i].pose.position.x
            self.path_poses[i, 1] = path.poses[i].pose.position.y
            self.path_poses[i, 2] = path.poses[i].pose.position.z
        ## calculate curvature:
        self.path_poses, self.looping = self.interpolate_path(self.path_poses[:,:3])
        print("path received")
        self.current_wp_index = 0
        self.goal = self.path_poses[self.current_wp_index, :3]
        
        path = navPath()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for i in range(len(self.path_poses)):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = self.path_poses[i,0]
            pose.pose.position.y = self.path_poses[i,1]
            pose.pose.position.z = self.path_poses[i,2]
            path.poses.append(pose)
        self.interpolated_path_pub.publish(path)
        self.controller.mppi.reset()
        self.goal_init = True

    def interpolate_path(self, target_WP):
        target_Vhat = np.zeros_like(target_WP) # 0 out everything
        max_index = len(target_WP) - 1
        
        for i in range(1,len(target_WP)-1): # all points except first and last
            V_prev = target_WP[i] - target_WP[i-1]
            V_next = target_WP[i+1] - target_WP[i]
            target_Vhat[i] = (V_next + V_prev)/np.linalg.norm(V_next + V_prev)
        target_Vhat[0] = target_WP[1] - target_WP[0]
        target_Vhat[0] /= np.linalg.norm(target_Vhat[0])
        target_Vhat[-1] = target_WP[-1] - target_WP[-2]
        target_Vhat[-1] /= np.linalg.norm(target_Vhat[-1])

        looping = False
        if np.linalg.norm(target_WP[0] - target_WP[-1]) < 2:
            looping = True ## this is to cause the system to reset wp index to 0 if loop is detected
            print("looping")
        N = 40
        wp_list = []
        for i in range(len(target_WP)-1):
            P0 = target_WP[i]
            P3 = target_WP[i+1]
            P1,P2 = get_Intermediate_Points_generic(P0,P3, target_Vhat[i], target_Vhat[i+1],2.0,compliment=False)
            bx,by,bz = get_bezier(P0,P1,P2,P3,float(N))
            for j in range(N):
                Curvature,Direction,Normal = get_CTN(P0,P1,P2,P3,float(j)/float(N))
                wp_list.append(np.array([bx[j], by[j], bz[j], Curvature]))

        return np.array(wp_list), looping


if __name__ == "__main__":
    rospy.init_node("hl_controller")
    config_name = "dawg_mppi.yaml"
    config_path = "/root/catkin_ws/src/dawg_core/config/" + config_name
    with open(config_path) as f:
        Config = yaml.safe_load(f)
    planner = dawg_HL_Control(Config)
    rospy.spin()
