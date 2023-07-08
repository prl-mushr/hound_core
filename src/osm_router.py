## this ros node listens to a waypoint topic from mavros and publishes a path from the start point to the end point
import rospy
from pyroutelib3 import Router # Import the router
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import WaypointList, Waypoint
import time

class OSM_Router:
    def __init__(self):
        ## create subscriber for waypoint messages and publisher for path messages
        self.router = Router("cycle") ## the car is similar to a cycle in it's motion constraints.

        self.waypoint_sub = rospy.Subscriber(
            "/mavros/mission/waypoints",
            WaypointList,
            self.waypoint_callback,
        )
        ## subscribe to local position and global position:
        self.local_pos_sub = rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            self.local_pos_callback,
        )
        self.global_pos_sub = rospy.Subscriber(
            "/mavros/global_position/global",
            PoseStamped,
            self.global_pos_callback,
        )

        self.path_pub = rospy.Publisher(
            "path",
            Path,
            queue_size=1,
        )

        self.gps_origin = (0,0)
        self.local_pos = None
        self.global_pos = None
        self.waypoint_list = None
        self.earthRadius = 6378145.0
        self.DEG_TO_RAD = 0.01745329252
        self.RAD_TO_DEG = 57.2957795131

    
    def waypoint_callback(self, msg):
        self.waypoint_list = msg.waypoints
        print("waypoint list received")
        print(self.waypoint_list)
        while self.global_pos is None or self.local_pos is None:
            time.sleep(1)
            print("waiting for global and local position")
        local_x = self.local_pos.position.x
        local_y = self.local_pos.position.y
        global_lat = self.global_pos.position.latitude
        global_lon = self.global_pos.position.longitude
        self.gps_origin = self.calcposLLH(global_lat, global_lon, -local_x, -local_y)
        print("gps origin is: ", self.gps_origin)
        last_lat = self.gps_origin[0]
        last_lon = self.gps_origin[1]

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for waypoint in self.waypoint_list:
            lat = waypoint.x_lat
            lon = waypoint.y_long
            X, Y = self.find_route(last_lat, last_lon, lat, lon, self.gps_origin[0], self.gps_origin[1])
            last_lat = lat
            last_lon = lon
            if X is None:
                print("no route found between waypoints:, ", last_lat, last_lon, lat, lon)
                return ## return if no route found
            for i in range(len(X)):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = X[i]
                pose.pose.position.y = Y[i]
                pose.pose.position.z = 0
                path.poses.append(pose)
                
        self.path_pub.publish(path)
    
    def local_pos_callback(self, msg):
        self.local_pos = msg.pose
        print("local position received")
        print(self.local_pos)
    
    def global_pos_callback(self, msg):
        self.global_pos = msg.pose
        print("global position received")
        print(self.global_pos)

    def calcposLLH(self, lat, lon, dX, dY):
        lat += dY / (self.earthRadius * self.DEG_TO_RAD )
        lon += dX / (self.earthRadius * np.cos(lat) * self.DEG_TO_RAD)
        return lat, lon

    def calcposNED(self, lat, lon, latReference, lonReference):
        Y = self.earthRadius * (lat - latReference) * self.DEG_TO_RAD
        X = self.earthRadius * np.cos(latReference) * (lon - lonReference) * self.DEG_TO_RAD
        return X, Y

    def find_route(self, start_lat,start_lon,end_lat,end_lon, home_lat, home_lon):
        start = self.router.findNode(start_lat, start_lon) # Find start and end nodes
        end = self.router.findNode(end_lat, end_lon)

        status, route = self.router.doRoute(start, end) # Find the route - a list of OSM nodes
        print(status)
        if status == 'success':
            print("found route")
            routeLatLons = list(map(self.router.nodeLatLon, route)) # Get actual route coordinates
        else:
            return None, None ## return empty arrays if no route found
        Y = np.array([i[0] for i in routeLatLons])
        X = np.array([i[1] for i in routeLatLons])

        X, Y = self.calcposNED(np.copy(Y), np.copy(X), home_lat, home_lon)
        x = []
        y = []
        for i in range(len(X)-1):
            vec = np.array([X[i+1], Y[i+1]]) - np.array([X[i], Y[i]])
            dist = np.linalg.norm(vec)
            N = int(dist/0.2)
            x.append(np.linspace(X[i], X[i+1], N))
            y.append(np.linspace(Y[i], Y[i+1], N))
        X = np.hstack(x)
        Y = np.hstack(y)

        return X,Y

# initialize ros node:
if __name__ == "__main__":
    rospy.init_node("osm_router")
    router = OSM_Router()
    rospy.spin()
    