# this file takes a rosbag, reads the GPS message on topic 'mavros/global_position/raw/fix' and writes the latitude and longitude to a yaml file.
# the raw fix messages come in at a very high rate, but we don't need all of them. We want the waypoints to be 'd' distance apart, so you must check the distance between the current waypoint and the last waypoint. If it is greater than 'd', then write the current waypoint to the yaml file.

import rosbag
import yaml
import argparse
import math
import numpy as np
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt


earthRadius = 6378145.0
DEG_TO_RAD = 0.01745329252
RAD_TO_DEG = 57.2957795131

# distance function
def LL2distance( lat, lon, latReference, lonReference):
    Y = earthRadius * (lat - latReference) * DEG_TO_RAD
    X = (
        earthRadius
        * np.cos(latReference * DEG_TO_RAD)
        * (lon - lonReference)
        * DEG_TO_RAD
    )
    # distance is:
    # sqrt(X^2 + Y^2)
    d = math.sqrt(X ** 2 + Y ** 2)
    return d

def calcPosNED(lat, lon, latRef, lonRef):
    Y = earthRadius * (lat - latRef) * DEG_TO_RAD
    X = (
        earthRadius
        * np.cos(latRef * DEG_TO_RAD)
        * (lon - lonRef)
        * DEG_TO_RAD
    )
    return X, Y

# use argparse to get the name of the rosbag. Assume rosbag is always in /root/catkin_ws/src/bags/
# use argparse to get the name of the yaml file. Assume yaml file is always in /root/catkin_ws/src/hound_core/waypoints/
# use argparse to get the distance between waypoints

# check if the number of arguments is correct
parser = argparse.ArgumentParser(description="rosbag to waypoints")
parser.add_argument("--rosbag", type=str, help="rosbag file", default="hound_17.bag")
parser.add_argument("--yaml", type=str, help="yaml file", default="waypoints_flat.yaml")
parser.add_argument("--distance", type=float, help="distance between waypoints", default=5.0)
args = parser.parse_args()

# open the rosbag
bag = rosbag.Bag("/root/catkin_ws/src/bags/" + args.rosbag)

# the yaml file does not exist, so create it
yaml_file = open("/root/catkin_ws/src/hound_core/waypoints/" + args.yaml, "w")

# initialize the last waypoint
last_lat = 0
last_lon = 0

# for debugging purposes, plot the waypoints using matplotlib

# create a list of lats and lons (separate) to plot the waypoints
lats = []
lons = []

# iterate through the rosbag
for topic, msg, t in bag.read_messages(topics=["/mavros/global_position/raw/fix"]):
    lat = msg.latitude
    lon = msg.longitude
    # if the distance between the current waypoint and the last waypoint is greater than the distance between waypoints
    if LL2distance(lat, lon, last_lat, last_lon) > args.distance:
        # write the current waypoint to the yaml file
        # waypoints are supposed to be in the format: waypoint_index: [latitude, longitude] (newline)
        yaml_file.write("waypoint_" + str(len(lats)) + ": [" + str(lat) + ", " + str(lon) + "]\n")
        # for debugging purposes, plot the waypoints using matplotlib and the basemap toolkit:
        lats.append(lat)
        lons.append(lon)
        # update the last waypoint
        last_lat = lat
        last_lon = lon

# close the yaml file
yaml_file.close()

# use calcPosNED to get the NED position of the waypoints and then plot that using matplotlib:
# use numpy to convert the lats and lons to numpy arrays
lats = np.array(lats)
lons = np.array(lons)
# now use calcPosNED to get the NED position of the waypoints
E, N = calcPosNED(lats, lons, lats[0], lons[0])

# for debugging purposes, plot the waypoints using matplotlib
plt.plot(E, N)
plt.axis("equal")
plt.show()
