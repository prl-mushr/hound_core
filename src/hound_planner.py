#!/usr/bin/env python3
import rospy
from pyroutelib3 import Router  # Import the router
import numpy as np
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from mavros_msgs.msg import WaypointList
from sensor_msgs.msg import NavSatFix
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from tf.transformations import euler_from_quaternion
import yaml


import rospy
import torch
from torch.utils.cpp_extension import load
import numpy as np
import time
import traceback
import cv2
from grid_map_msgs.msg import GridMap


class IGHAStar:
    def __init__(self, configs):
        self.path = None
        self.success = False
        self.completed = True
        self.expansion_counter = 0
        self.layers = None
        self.index = None

        self.grid_map_sub = rospy.Subscriber(
            "/elevation_mapping/elevation_map_cropped_cv2",
            GridMap,
            self.grid_map_callback,
        )
        self.map_res = configs["experiment_info_default"]["node_info"]["map_res"]
        self.expansion_limit = configs["experiment_info_default"]["max_expansions"]
        self.hysteresis = configs["experiment_info_default"]["hysteresis"]
        self.offset = None
        self.map_size_px = None
        self.map_center = None
        self.map_init = False
        self.heightmap = None
        self.normal = None
        self.costmap = None
        self.start = torch.zeros(4, dtype=torch.float32)
        self.goal = torch.zeros(4, dtype=torch.float32)
        self.graph = {}
        self.create_planner(configs)

    def create_planner(self, configs):
        env_name = "kinodynamic"
        env_macro = {
            'simple': '-DUSE_SIMPLE_ENV',
            'kinematic': '-DUSE_KINEMATIC_ENV',
            'kinodynamic': '-DUSE_KINODYNAMIC_ENV',
        }[env_name]
        folder_path = configs["folder_path"]
        cpp_path = f'{folder_path}/src/ighastar.cpp'
        cuda_path = f'{folder_path}/src/{env_name}.cu'
        header_path = f'{folder_path}/src'

        kernel = load(
            name="ighastar",
            sources=[cpp_path, cuda_path],
            extra_include_paths=[header_path],
            extra_cflags=['-std=c++17', '-O3', env_macro],
            extra_cuda_cflags=['-O3'],
            verbose=True,
        )
        self.planner = kernel.IGHAStar(configs, False)
        print("planner loaded")

    def grid_map_callback(self, grid_map):
        self.grid_map = grid_map
        if self.index is None or self.layers is None:
            self.layers = self.grid_map.layers
            self.index = self.layers.index("elevation")
        cent = self.grid_map.info.pose.position
        matrix = self.grid_map.data[self.index]
        self.temp_heightmap = np.float32(
            cv2.flip(
                np.reshape(
                    matrix.data,
                    (matrix.layout.dim[1].size, matrix.layout.dim[0].size),
                    order="F",
                ).T,
                -1,
            )
        )
        if np.any(np.isnan(self.temp_heightmap)):
            self.map_init = False
            print("got nan")
        else:
            self.map_center = np.array([cent.x, cent.y, cent.z])
            self.process_grid_map()
            self.map_init = True

    def process_grid_map(self):
        self.heightmap = np.copy(self.temp_heightmap)
        self.normal = self.generate_normal(self.heightmap)
        if self.map_size_px == None:
            self.map_size_px = self.heightmap.shape[0]
        ## upscaling this AFTER calculating the surface normal.
        self.map_center[2] = self.heightmap[self.map_size_px // 2, self.map_size_px // 2]
        self.heightmap -= self.map_center[2]
        self.costmap = self.generate_costmap_from_BEVmap(self.normal)
        self.bitmap = torch.ones((self.map_size_px, self.map_size_px, 2), dtype=torch.float32)
        self.bitmap[..., 0] = torch.from_numpy(self.costmap)
        self.bitmap[..., 1] = torch.from_numpy(self.heightmap)
        self.offset = self.map_res * np.array(self.bitmap.shape[:2]) * 0.5

    def generate_costmap_from_BEVmap(self, normal, costmap_cosine_thresh=np.cos(np.radians(30))):
        dot_product = normal[:, :, 2]
        costmap = np.where(dot_product >= costmap_cosine_thresh, 255, 0).astype(np.float32)
        return costmap

    def generate_normal(self, elev, k=3):
        # use sobel filter to generate the normal map:
        norm = np.copy(elev)
        dzdx = -cv2.Sobel(norm, cv2.CV_32F, 1, 0, ksize=k)
        dzdy = -cv2.Sobel(norm, cv2.CV_32F, 0, 1, ksize=k)
        dzdz = np.ones_like(norm)
        normal = np.stack((dzdx, dzdy, dzdz), axis=2)
        norm = np.linalg.norm(normal, axis=2, keepdims=True)
        normal = normal / norm
        return normal
    
    def clip_goal_to_map(self, num_samples=20):
        H, W = self.bitmap.shape[-2:]
        
        x0, y0 = self.start[:2]
        x1, y1 = self.goal[:2]

        # Generate t in [0, 1]
        t_vals = torch.linspace(0, 1, num_samples, device=self.bitmap.device)

        # Line points: start + t * (goal - start)
        dx = x1 - x0
        dy = y1 - y0
        xs = x0 + t_vals * dx
        ys = y0 + t_vals * dy

        # Check which are inside map bounds
        valid = (xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)

        if torch.any(valid):
            last_valid_idx = torch.where(valid)[0][-1]
            self.goal[0] = xs[last_valid_idx]
            self.goal[1] = ys[last_valid_idx]
        else:
            # All points are out of bounds; snap to start
            self.goal[0] = x0
            self.goal[1] = y0

    def plan(self, start_state, goal_, stop = False):
        # pray to god I don't need a thread lock here
        if not self.map_init:
            return
        bitmap = torch.clone(self.bitmap)
        map_center = np.copy(self.map_center)[:2]
        self.start[:2] = torch.from_numpy(start_state[:2] + self.offset - map_center)
        self.goal[:2] = torch.from_numpy(goal_[:2] + self.offset - map_center)
        # clip the goal to the map size but dont change its aspect (angle)
        self.clip_goal_to_map()

        self.start[2] = start_state[2]
        self.start[3] = start_state[3]
        self.goal[2] = goal_[2]
        self.goal[3] = 0
        # drawmap = bitmap[..., 0].numpy()
        # drawmap = cv2.cvtColor(drawmap, cv2.COLOR_GRAY2RGB)
        # draw a circle at the start and goal position, use map_res to get pixel locations:
        # start = np.round(self.start[:2].numpy() / self.map_res).astype(np.int32)
        # goal = np.round(self.goal[:2].numpy() / self.map_res).astype(np.int32)
        # draw a circle at the start and goal position
        # cv2.circle(drawmap, (start[0], start[1]), 5, (255, 0, 0), -1)
        # cv2.circle(drawmap, (goal[0], goal[1]), 5, (0, 255, 0), -1)
        # cv2.imshow("map", drawmap)
        # cv2.waitKey(1)
        # print(self.goal, self.start, map_center, self.offset, self.map_size_px, self.map_size_px*self.map_res)
        # path = np.array([])
        # return PlanResult(path, path, self.expansion_counter, self.expansion_counter, 100)
        # dx = self.goal[0] - self.start[0]
        # dy = self.goal[1] - self.start[1]
        # self.goal[2] = torch.atan2(dy, dx)
        self.goal[3] = 0.0 if stop else 4.0
        print("start:", self.start.numpy())
        print("goal:", self.goal.numpy())
        now = time.perf_counter()
        # ignore goal validity with last flag
        success = self.planner.search(self.start, self.goal, bitmap, self.expansion_limit, self.hysteresis, True)

        end = time.perf_counter()

        if success:
            path = self.planner.get_best_path().numpy()
            path = np.flip(path, axis=0)
            path[..., :2] -= self.offset
            path[..., :2] += map_center
            return path
        else:
            return []


class Planner:
    def __init__(self, Config):
        ## create subscriber for waypoint messages and publisher for path messages
        self.planner = IGHAStar(Config["Planner_config"])
        self.wp_radius = Config["wp_radius"]

        self.waypoint_sub = rospy.Subscriber(
            "/mavros/mission/waypoints",
            WaypointList,
            self.waypoint_callback,
            queue_size=10,
        )
        ## subscribe to local position and global position:
        self.local_pos_sub = rospy.Subscriber(
            "/mavros/local_position/odom",
            Odometry,
            self.local_pos_callback,
        )
        self.global_pos_sub = rospy.Subscriber(
            "/mavros/global_position/global",
            NavSatFix,
            self.global_pos_callback,
        )

        self.path_pub = rospy.Publisher("/path", Path, queue_size=1, latch=True)
        self.diagnostics_pub = rospy.Publisher(
            "/planner_diagnostics", DiagnosticArray, queue_size=2
        )

        self.gps_origin = (0, 0)
        self.local_pos = None
        self.global_pos = None
        self.waypoint_list = None
        self.local_waypoints = None
        self.earthRadius = 6378145.0
        self.DEG_TO_RAD = 0.01745329252
        self.RAD_TO_DEG = 57.2957795131

        self.generate_path = False
        config_file = "/root/catkin_ws/src/hound_core/config/hound_mppi_real.yaml"
        # use yaml to read the fields "use_yaml" and "waypoint_file"
        with open(config_file) as f:
            Config = yaml.safe_load(f)
        # timer callback:
        self.timer = rospy.Timer(rospy.Duration(0.5), self.plan_loop)

    def diagnostic_publisher(self, status):
        diagnostics_array = DiagnosticArray()
        diagnostics_status = DiagnosticStatus()
        diagnostics_status.name = "planner"
        diagnostics_status.level = status

        diagnostics_array.status.append(diagnostics_status)
        self.diagnostics_pub.publish(diagnostics_array)

    def waypoint_callback(self, msg):        
        self.waypoint_list = msg.waypoints
        while self.global_pos is None or self.local_pos is None:
            time.sleep(1)
            self.diagnostic_publisher(1)
        self.diagnostic_publisher(0)
        local_x = self.local_pos.position.x
        local_y = self.local_pos.position.y
        global_lat = self.global_pos.latitude
        global_lon = self.global_pos.longitude
        self.gps_origin = self.calcposLLH(global_lat, global_lon, -local_x, -local_y)
        # print("gps origin: ", self.gps_origin)
        self.local_waypoints = []
        for waypoint in self.waypoint_list:
            if waypoint.frame != 3:
                continue
            lat = waypoint.x_lat
            lon = waypoint.y_long
            # generate X,Y locations using calcposNED and append to path
            X, Y = self.calcposNED(lat, lon, self.gps_origin[0], self.gps_origin[1])
            self.local_waypoints.append(X, Y)
        self.local_waypoints = self.set_headings(self.local_waypoints)

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

    def local_pos_callback(self, msg):
        self.local_pos = msg.pose
        self.obtain_state(msg)
    
    def plan_loop(self):
        if self.local_waypoints is not None and len(self.local_waypoints):
            # do something
            plan_state = np.array([self.state[0], self.state[1], self.state[5], np.linalg.norm(self.state[6:8])])
            goal = self.local_waypoints[0]
            self.planner(plan_state, goal, stop=len(self.local_waypoints) == 1)
            # reduce length of local waypoints:
            dist = np.linalg.norm(self.local_waypoints[0] - plan_state[:2])
            if dist < self.wp_radius:
                self.local_waypoints = self.local_waypoints[1:]

    def set_headings(self, local_waypoints):
        target_Vhat = np.zeros_like(local_waypoints)  # 0 out everything
        max_index = len(local_waypoints) - 1

        for i in range(1, len(local_waypoints) - 1):  # all points except first and last
            V_prev = local_waypoints[i] - local_waypoints[i - 1]
            V_next = local_waypoints[i + 1] - local_waypoints[i]
            target_Vhat[i] = (V_next + V_prev) / np.linalg.norm(V_next + V_prev)
        target_Vhat[0] = local_waypoints[1] - local_waypoints[0]
        target_Vhat[0] /= np.linalg.norm(target_Vhat[0])
        target_Vhat[-1] = local_waypoints[-1] - local_waypoints[-2]
        target_Vhat[-1] /= np.linalg.norm(target_Vhat[-1])

        waypoints = np.zeros((len(local_waypoints), 4))
        waypoints[:, :2] = local_waypoints
        waypoints[:, 2] = np.arctan2(target_Vhat[:, 1], target_Vhat[:, 0])
        waypoints[:, 3] = 2.0
        return 

    def global_pos_callback(self, msg):
        self.global_pos = msg

    def calcposLLH(self, lat, lon, dX, dY):
        lat += dY / (self.earthRadius * self.DEG_TO_RAD)
        lon += dX / (self.earthRadius * np.cos(lat * self.DEG_TO_RAD) * self.DEG_TO_RAD)
        return lat, lon

    def calcposNED(self, lat, lon, latReference, lonReference):
        Y = self.earthRadius * (lat - latReference) * self.DEG_TO_RAD
        X = (
            self.earthRadius
            * np.cos(latReference * self.DEG_TO_RAD)
            * (lon - lonReference)
            * self.DEG_TO_RAD
        )
        return X, Y


# initialize ros node:
if __name__ == "__main__":
    rospy.init_node("hound_planner")
    config_name = "hound_mppi_planner.yaml"
    config_path = "/root/catkin_ws/src/hound_core/config/" + config_name
    with open(config_path) as f:
        Config = yaml.safe_load(f)
    planner = Planner(Config)
    while rospy.not_shutdown():
        planner.plan_loop()