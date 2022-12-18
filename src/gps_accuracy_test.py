#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW
from nav_msgs.msg import Odometry
import numpy as np
import math as m
import matplotlib.pyplot as plt
import time

rospy.init_node('vesc_to_odom_node', anonymous=True)

def calcposNED(lat, lon, latReference, lonReference):
	earthRadius = 6378145.0
	lat /= 57.3
	lon /= 57.3
	latReference /= 57.3
	lonReference /= 57.3
	posNEDr = np.zeros(3)
	Y = earthRadius * (lat - latReference)
	X = earthRadius * np.cos(latReference) * (lon - lonReference)
	return X, Y

def rotate(x,y, th):
    R = np.zeros((2,2))
    ct, st = m.cos(th), m.sin(th)
    R[0,0], R[0,1], R[1,0], R[1,1] = ct, -st, st, ct
    X = np.array([x,y])
    V = np.matmul(R, X)
    return V[0], V[1]

gps_raw_lat = []
gps_raw_lon = []
gps_lat = []
gps_lon = []
heading = []

start = np.array([47.653974, -122.307231])
end = np.array([47.654131, -122.306735])

gps_ground_lat = np.arange(start[0], end[0], 0.000001)
N = float(len(gps_ground_lat))
resolution = m.fabs(start[1] -end[1])/N
gps_ground_lon = np.arange(start[1], end[1], resolution)
lat_Ref = gps_ground_lat[0]
lon_Ref = gps_ground_lon[0]

gps_ground_X, gps_ground_Y = calcposNED(gps_ground_lat, gps_ground_lon, lat_Ref, lon_Ref)
theta = m.atan2(gps_ground_Y[-1],gps_ground_X[-1])
gps_ground_X, gps_ground_Y = rotate(gps_ground_X, gps_ground_Y, -theta)

def gps_raw_cb(msg):
    gps_raw_lat.append(float(msg.lat)*1e-7)
    gps_raw_lon.append(float(msg.lon)*1e-7)

def gps_cb(msg):
    gps_lat.append(msg.latitude)
    gps_lon.append(msg.longitude)

def odom_cb(msg):
    orientation = msg.pose.pose.orientation
    yaw = m.atan2(2*(orientation.w*orientation.z + orientation.x*orientation.y), 1 - 2*(orientation.y**2 + orientation.z**2))
    yaw += 2.0/57.3
    heading.append(yaw)
    

gps_raw_sub = rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, gps_raw_cb, queue_size = 10)
gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_cb, queue_size = 10)
odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, odom_cb, queue_size = 10)

time.sleep(20)
gps_lat = np.array(gps_lat)
gps_lon = np.array(gps_lon)
gps_raw_lat = np.array(gps_raw_lat)
gps_raw_lon = np.array(gps_raw_lon)
heading = np.array(heading)

gps_X, gps_Y = calcposNED(gps_lat, gps_lon, lat_Ref, lon_Ref)
gps_raw_X, gps_raw_Y = calcposNED(gps_raw_lat, gps_raw_lon, lat_Ref, lon_Ref)

X,Y = rotate(gps_X, gps_Y, -theta)
X_raw, Y_raw = rotate(gps_raw_X, gps_raw_Y, -theta)

heading -= heading[0] ## make heading relative to first measurement
X_heading = X[-len(heading):]

plt.style.use('tableau-colorblind10')

plt.title("GPS error")
plt.ylabel("error in meters")
plt.xlabel("position along reference (straight) trajectory in meters")
plt.plot(X_raw, Y_raw, label="raw GPS position")
plt.plot(X, Y, label="filtered GPS position")
plt.plot(X, np.zeros_like(X), label="reference straight line")
plt.plot(X_heading, heading, label="relative heading in radians")

plt.legend()
plt.show()
print("filtered bias: ",np.mean(Y), " meters")
print("filtered variance: ", np.var(Y))
print("raw bias: ",np.mean(Y_raw), " meters")
print("raw variance: ", np.var(Y_raw))
Y_sorted = np.sort(np.fabs(Y))
Y_raw_sorted = np.sort(np.fabs(Y_raw))

plt.xlabel("error")
plt.ylabel("p(error)")
plt.plot(Y_sorted, Y_sorted.cumsum()/Y_sorted.sum(), label="cdf of filtered")
plt.plot(Y_raw_sorted, Y_raw_sorted.cumsum()/Y_raw_sorted.sum(), label="cdf of raw")
plt.xlim(0, max(Y_sorted[-1], Y_raw_sorted[-1]))
plt.ylim(0,1.1)
plt.legend()
plt.show()
