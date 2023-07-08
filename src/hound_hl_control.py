#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import message_filters
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion


"""
A high level control class that subscribes to the grid map, odometry, and imu topics, and a path and goal topic
The imu and odometry topics are supposed to be synchronized using ros message filters.
The goal is received as a pose message, and the path is received as a path message.
"""

## TODO: a failsafe that kicks in when we don't receive a map/state
## if state is older than 0.05 seconds, then we stop the car
## if the map is older than 0.1 seconds, then we stop the car

class Hound_HL_Control:
    def __init__(self):
        ## state variables
        self.odom = None
        self.imu = None
        self.state_init = False
        ## map variables
        self.map_init = False
        self.map_res = 0.1
        self.map_size = 512
        self.map_elev = None
        self.map_norm = None
        self.map_cost = None
        self.map_cent = None
        self.layers = None
        ## goal variables:
        self.goal = None
        self.goal_init = False
        self.path_poses = None
        self.path_cte = 1.0 ## 1 meter cross track error limit
        self.generate_cost_from_path = False ## if we want to use the waypoints for costmap generation
        self.waypoint_index = 0
        self.wp_radius = 2.0
        self.state = np.zeros(17)

        self.max_speed = 5.0
        self.max_steering = 0.38

        # initialize the odometry and imu subscribers:
        # ros message filter for synchronizing the odometry and imu messages:
        odom_sub = message_filters.Subscriber("/mavros/local_position/odom", Odometry)
        imu_sub = message_filters.Subscriber("/mavros/imu/data_raw", Imu)
        ts = message_filters.ApproximateTimeSynchronizer([odom_sub, imu_sub], 2, 0.01)
        ts.registerCallback(self.state_callback)

        self.grid_map_sub = rospy.Subscriber(
            "/grid_map_occlusion_inpainting/all_grid_map",
            GridMap,
            self.grid_map_callback,
        )
        self.goal_sub = rospy.Subscriber(
            "goal_in",
            PoseStamped,
            self.goal_callback,
        )
        self.path_sub = rospy.Subscriber(
            "path",
            Path,
            self.path_callback,
        )
        ## set up control publisher:
        self.control_pub = rospy.Publisher(
            "ackermann_cmd", AckermannDriveStamped, queue_size=1
        )
        ## set up the marker publisher:
        self.marker_pub = rospy.Publisher(
            "marker", MarkerArray, queue_size=1
        )
        ## initialize controller:
        print("starting")

    def update_goal(self):
        dist = np.linalg.norm(self.goal[:2] - self.state[:2])
        if(dist < self.wp_radius):
            self.waypoint_index += 1
            self.goal = self.path_poses[self.waypoint_index]

    def step(self):
        if self.state_init and self.map_init:
            ## generate the control message:
            self.state = self.obtain_state()
            self.update_goal()
            # ctrl = self.control.update(self.state, self.goal, self.map_elev, self.map_norm, self.map_cost, self.map_cent)
            # self.state[15:17] = ctrl # last applied action
            print(self.goal)
            ctrl = np.array([0.1, 0.2])
            self.send_ctrl(ctrl)
            ## publish the markers
        else:
            print("state: ",self.state_init, "map_init: ", self.map_init, "goal_init: ", self.goal_init)

    def send_ctrl(self, ctrl):
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = "base_link"
        control_msg.drive.steering_angle = ctrl[0] * self.max_steering
        control_msg.drive.speed = ctrl[1] * self.max_speed
        self.control_pub.publish(ctrl)

    def obtain_state(self):
        ## obtain the state from the odometry and imu messages:
        state = np.zeros(17)
        quaternion = (
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        )
        rpy = euler_from_quaternion(quaternion)
        state[0] = self.odom.pose.pose.position.x
        state[1] = self.odom.pose.pose.position.y
        state[2] = self.odom.pose.pose.position.z
        state[3] = rpy[0]
        state[4] = rpy[1]
        state[5] = rpy[2]
        state[6] = self.odom.twist.twist.linear.x
        state[7] = self.odom.twist.twist.linear.y
        state[8] = self.odom.twist.twist.linear.z
        state[9] = self.imu.linear_acceleration.x
        state[10] = self.imu.linear_acceleration.y
        state[11] = self.imu.linear_acceleration.z
        state[12] = self.imu.angular_velocity.x
        state[13] = self.imu.angular_velocity.y
        state[14] = self.imu.angular_velocity.z
        
        return state

    def state_callback(self, odom, imu):
        if not self.state_init:
            self.state_init = True
        self.odom = odom
        self.imu = imu
        self.step()

    def grid_map_callback(self, grid_map):
        if not self.map_init:
            # initialize the map and set the "one time" variables:
            self.map_init = True
            self.map_res = grid_map.info.resolution
            self.map_size = int(grid_map.info.length_x / self.map_res)
            self.layers = grid_map.layers

        data = np.array(grid_map.data)
        map_elev = np.array(data[5].data, dtype=np.float32).reshape(
            self.map_size, self.map_size
        )
        self.map_elev = cv2.flip(map_elev, -1)
        # center of the grid map:
        self.map_cent = grid_map.info.pose.position
        self.map_norm = self.generate_normal(self.map_elev, self.map_res)
        ## generate the cost map:
        if(self.path_poses is not None and self.generate_cost_from_path):
            self.map_cost = self.generate_cost(self.map_size, self.map_res, self.map_cent, self.path_poses, self.path_cte)
        else:
            self.map_cost = np.zeros((self.map_size, self.map_size), dtype=np.float32)

    def generate_normal(self, elev, res):
        # use sobel filter to generate the normal map:
        size = elev.shape[0]
        norm = np.copy(elev)
        norm = cv2.resize(norm, (size*2, size*2), cv2.INTER_AREA)
        norm = cv2.GaussianBlur(norm, (3, 3), 0)
        dzdx = -cv2.Sobel(norm, cv2.CV_64F, 1, 0, ksize=3)
        dzdy = -cv2.Sobel(norm, cv2.CV_64F, 0, 1, ksize=3)
        dzdz = np.ones_like(norm)
        normal = np.stack((dzdx, dzdy, dzdz), axis=2)
        norm = np.linalg.norm(normal, axis=2, keepdims=True)
        normal = normal / norm  ## normalize the normal vector lol
        normal = cv2.resize(normal, (size, size), cv2.INTER_AREA)
        return normal

    def generate_cost(self, size, res, cent, _poses, track_width):
        '''
        the idea here is to consider the end point of the path as a "goal"
        and use the points along the path to construct a "costmap" where we draw circles around the points
        and then blur them out to give the optimizer some sense of "distance" from the path
        '''
        r = int(track_width / res)
        r2 = r * 2
        costmap = np.ones((size, size), dtype=np.float32)
        ## track width is in meters, we need to convert it to pixels:
        poses = np.copy(_poses)
        poses -= cent
        poses *= 1.0 / res
        poses += size / 2.0
        poses = poses.astype(np.int32)
        poses = poses[np.where(poses[:,0] >= 0 and poses[:,0] < size and poses[:,1] >= 0 and poses[:,1] < size)]
        pts = poses[:,:2].reshape((-1, 1, 2))
        image = cv2.polylines(image, [pts], False, 0, r)
        costmap = cv2.blur(costmap, (r2, r2))
        return costmap

    # goal callback:
    def goal_callback(self, goal):
        ''' 
        goal is a pose message
        '''
        self.goal = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])
        self.goal_init = True # this variable will be set to false when the goal is reached
        rospy.loginfo("goal received, ignoring previously assigned path")
        self.path_poses = None

    # path callback:
    def path_callback(self, path):
        '''
        we expect the path to have a resolution of 0.2 meters or less.
        '''
        ## get the poses from the path and store them into a numpy array:
        self.path_poses = np.zeros((len(path.poses), 3), dtype=np.float32)
        for i in range(len(path.poses)):
            self.path_poses[i, 0] = path.poses[i].pose.position.x
            self.path_poses[i, 1] = path.poses[i].pose.position.y
            self.path_poses[i, 2] = path.poses[i].pose.position.z
        rospy.loginfo("path received, ignoring previously assigned goal")
        self.waypoint_index = 0
        self.goal = self.path_poses[self.waypoint_index]
        self.goal_init = True
    

if __name__ == "__main__":
    rospy.init_node("hl_controller")
    planner = Hound_HL_Control()
    rospy.spin()
