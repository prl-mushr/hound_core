#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import traceback

rospy.init_node("optical_flow_test", anonymous=True)

bridge = CvBridge()

init = False
prevgray = []

def image_callback(img_msg):
    global prevgray, init
    try:
        img = bridge.imgmsg_to_cv2(img_msg, "bgr8")# [:,:,:3]
        now = time.time()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.resize(img, (160,120))
        if(init == True):
            #t = np.arange(256)
            #a = []
            #for i in range(1024):
            #    a.append(np.sin(i*0.01*t))
            #a = np.array(a)
            now = time.time()
            #for i in range(1024):
            #    fft = np.fft.fft(a[i])
            flow = cv2.calcOpticalFlowFarneback(prevgray, img,None, 0.5, 3, 40, 1, 5, 1.2, 0)
            dt = time.time() - now
            print(dt*1e3)
        init = True
        prevgray = img

    except Exception:
        print(traceback.format_exc())

img_sub = rospy.Subscriber("/camera/infra1/image_rect_raw", Image, image_callback, queue_size = 1)

while not rospy.is_shutdown():
    time.sleep(1)

