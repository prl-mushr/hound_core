#!/usr/bin/env python3
import rospy
from pyroutelib3 import Router  # Import the router
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import WaypointList
from sensor_msgs.msg import NavSatFix
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class OSM_Router:
    def __init__(self):
        ## create subscriber for waypoint messages and publisher for path messages
        self.router = Router(
            "cycle"
        )  ## the car is similar to a cycle in it's motion constraints.

        self.waypoint_sub = rospy.Subscriber(
            "/mavros/mission/waypoints",
            WaypointList,
            self.waypoint_callback,
            queue_size=10,
        )
        ## subscribe to local position and global position:
        self.local_pos_sub = rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
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
        self.earthRadius = 6378145.0
        self.DEG_TO_RAD = 0.01745329252
        self.RAD_TO_DEG = 57.2957795131

        self.generate_path = False

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
        last_lat = self.gps_origin[0]
        last_lon = self.gps_origin[1]

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for waypoint in self.waypoint_list:
            if waypoint.frame != 3:
                continue
            lat = waypoint.x_lat
            lon = waypoint.y_long
            if self.generate_path:
                X, Y = self.find_route(
                    last_lat, last_lon, lat, lon, self.gps_origin[0], self.gps_origin[1]
                )
                last_lat = lat
                last_lon = lon
                if X is None:
                    print(
                        "no route found between waypoints:, ",
                        last_lat,
                        last_lon,
                        lat,
                        lon,
                    )
                    return  ## return if no route found
                for i in range(len(X)):
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = X[i]
                    pose.pose.position.y = Y[i]
                    pose.pose.position.z = 0
                    path.poses.append(pose)
            else:
                # generate X,Y locations using calcposNED and append to path
                X, Y = self.calcposNED(lat, lon, self.gps_origin[0], self.gps_origin[1])
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = X
                pose.pose.position.y = Y
                pose.pose.position.z = 2
                path.poses.append(pose)

        self.path_pub.publish(path)

    def local_pos_callback(self, msg):
        self.local_pos = msg.pose

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

    def find_route(self, start_lat, start_lon, end_lat, end_lon, home_lat, home_lon):
        start = self.router.findNode(start_lat, start_lon)  # Find start and end nodes
        end = self.router.findNode(end_lat, end_lon)

        status, route = self.router.doRoute(
            start, end
        )  # Find the route - a list of OSM nodes
        print(status)
        if status == "success":
            print("found route")
            routeLatLons = list(
                map(self.router.nodeLatLon, route)
            )  # Get actual route coordinates
        else:
            return None, None  ## return empty arrays if no route found
        Y = np.array([i[0] for i in routeLatLons])
        X = np.array([i[1] for i in routeLatLons])

        X, Y = self.calcposNED(np.copy(Y), np.copy(X), home_lat, home_lon)
        x = []
        y = []
        for i in range(len(X) - 1):
            vec = np.array([X[i + 1], Y[i + 1]]) - np.array([X[i], Y[i]])
            dist = np.linalg.norm(vec)
            N = int(dist / 0.2)
            x.append(np.linspace(X[i], X[i + 1], N))
            y.append(np.linspace(Y[i], Y[i + 1], N))
        X = np.hstack(x)
        Y = np.hstack(y)

        return X, Y


# initialize ros node:
if __name__ == "__main__":
    rospy.init_node("osm_router")
    router = OSM_Router()
    rospy.spin()
