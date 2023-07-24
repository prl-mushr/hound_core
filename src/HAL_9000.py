#!/usr/bin/env python3
# Import ROS libraries and messages
import os
os.system('usbreset "ChibiOS/RT Virtual COM Port"') ## this gets executed with 0 delay
import rospy
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rostopic import ROSTopicHz
import yaml
from mavros_msgs.msg import PlayTuneV2, RCIn, GPSRAW
import subprocess, shlex, psutil
from vesc_msgs.msg import VescStateStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import tf
import numpy as np

class hal():
    def __init__(self, config_file):
        with open(config_file) as f:
            config = yaml.safe_load(f)
            print("configs found")
        if(config == None):
            print("no config found, make sure the config file path is correct")
            exit()

        self.mavros_topic = config["mavros_topic"]
        self.mavros_action = config["mavros_action"]
        self.mavros_hz = ROSTopicHz(3)

        self.camera_topic = config["camera_topic"]
        self.camera_hz = ROSTopicHz(3)

        sub_mavros = rospy.Subscriber(self.mavros_topic, rospy.AnyMsg, self.mavros_hz.callback_hz)
        sub_camera = rospy.Subscriber(self.camera_topic, rospy.AnyMsg, self.camera_hz.callback_hz)

        sub_channel = rospy.Subscriber("mavros/rc/in", RCIn, self.channel_cb, queue_size=2)
        sub_voltage = rospy.Subscriber("sensors/core", VescStateStamped, self.voltage_cb, queue_size = 1)
        sub_gps = rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, self.GPS_cb, queue_size = 1)
        pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        
        self.lp = lg.LaserProjection()
        self.pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
        self.last_pose_time = None

        self.notification_pub = rospy.Publisher("/mavros/play_tune", PlayTuneV2, queue_size =10)
        
        self.bagdir = config["bagdir"]
        record_command_file = config["record_command_file"]
        with open(record_command_file, 'r') as f:
            self.record_command = f.read()
        
        self.mavros_init = False
        self.camera_init = False
        
        self.recording_state = False
        self.rosbag_proc = None
        self.GPS_status = False
        self.mavros_action = config["mavros_action"]

        self.diagnostics_pub = rospy.Publisher("/SOC_diagnostics", DiagnosticArray, queue_size=2)
        
        time.sleep(15)
        os.system(self.mavros_action) ## first attempt at getting mavros to run @50 Hz
        self.main_loop()

    def publish_notification(self, message):
        playtune = PlayTuneV2()
        playtune.format = 1;
        if(message == "low level ready"):
            playtune.tune = "MLO2L2A"
        elif(message == "low battery"):
            playtune.tune = "MSO3L8dddP8ddd"
        elif(message == "record start"):
            playtune.tune = "ML O3 L8 CD"
        elif(message == "record stop"):
            playtune.tune = "ML O3 L8 DC"
        elif(message == "GPS good"):
            playtune.tune = "MSO3L8dP8d"
        elif(message == "GPS bad"):
            playtune.tune = "MSO3L8ddP8dd"
        elif(message == "camera ready"):
            playtune.tune = "MLO2L2C"
        self.notification_pub.publish(playtune)
    
    def start_recording(self):
        self.command = 'rosbag record --split --duration=30s -O ' + self.bagdir + 'hound ' + self.record_command
        print("executing: ", self.command)
        self.command = shlex.split(self.command)
        self.rosbag_proc = subprocess.Popen(self.command)
        self.publish_notification("record start")
    
    def stop_recording(self):
        for proc in psutil.process_iter():
            if "record" in proc.name() and set(self.command[2:]).issubset(proc.cmdline()):
                proc.send_signal(subprocess.signal.SIGINT)

        self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
        self.publish_notification("record stop")
        files = os.listdir(self.bagdir)
        files.sort(key = lambda x: os.path.getmtime(self.bagdir+'/'+x))
        for i in range(len(files)):
            source = self.bagdir + '/' + files[i]
            dest = self.bagdir + '/' + 'hound_' + str(i) + '.bag'
            os.rename(source, dest)
    
    def voltage_cb(self, msg):
        if(msg.state.voltage_input < 14.0):
            self.publish_notification("low battery")
            time.sleep(1)
    
    def GPS_cb(self, msg):
        if(msg.satellites_visible >= 16 and msg.h_acc <= 1000 and self.GPS_status == False):
            self.publish_notification("GPS good")
            self.GPS_status = True
            time.sleep(1)
        elif( (msg.satellites_visible < 16 or msg.h_acc > 1000) and self.GPS_status == True):
            self.publish_notification("GPS bad")
            self.GPS_status = False
            time.sleep(1)
    
    def channel_cb(self, rc):
        try:
            if(len(rc.channels) == 0 ):
                return
            stick = rc.channels[3]
            if(self.recording_state == False and stick > 1900):
                print("start recording")
                self.start_recording()
                self.recording_state = True
            elif(self.recording_state == True and stick < 1100):
                print("stop recording")
                self.stop_recording()
                self.recording_state = False
        except:
            pass

    def scan_cb(self, msg):
        pc2_msg = self.lp.projectLaser(msg)
        if(self.last_pose_time != None):
            pc2_msg.header.stamp = self.last_pose_time
            self.pc_pub.publish(pc2_msg)

    def pose_cb(self, msg):
        br = tf.TransformBroadcaster()
        pos = msg.pose.position
        rot = msg.pose.orientation
        br.sendTransform((pos.x, pos.y, pos.z),
                         (rot.x, rot.y, rot.z, rot.w),
                         msg.header.stamp,
                         "base_link",
                         "map")

        br.sendTransform((0.15, 0.0, 0.02),
                         (0, 0, 0, 1),
                         msg.header.stamp,
                         "camera_depth_frame",
                         "base_link")

        br.sendTransform((0, 0, 0),
                         (0, 0, 0, 1),
                         msg.header.stamp,
                         "odom",
                         "map")

        br.sendTransform((0, 0, 0),
                         (-0.5, 0.5, -0.5, 0.5),
                         msg.header.stamp,
                         "camera_depth_optical_frame",
                         "camera_depth_frame")

        br.sendTransform((0.04, 0, -0.1),
                         (0, 0, 0, 1),
                         msg.header.stamp,
                         "laser_frame",
                         "base_link")
        self.last_pose_time = msg.header.stamp

    def publish_diagnostics(self):
        diagnostics_array = DiagnosticArray()
        diagnostics_status = DiagnosticStatus()
        diagnostics_status.name = 'SOC'

        temp = os.popen("cat /sys/devices/virtual/thermal/thermal_zone*/temp").readlines()
        avg_temp = []
        for t in temp:
            avg_temp.append(float(t.strip('\n'))*1e-3)
        avg_temp = np.round(np.mean(np.array(avg_temp)),2)

        avg_cpu = []
        temp = os.popen('cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq').readlines()
        for t in temp:
            avg_cpu.append(float(t.strip('\n'))*1e-6)
        avg_cpu = np.round(np.mean(np.array(avg_cpu)),2)

        avg_gpu = 1e-9*float(os.popen('cat /sys/devices/17000000.ga10b/devfreq/17000000.ga10b/max_freq').read().strip('\n'))
        avg_gpu = np.round(avg_gpu, 2)

        diagnostics_status.level = 0

        diagnostics_status.values.append(KeyValue(key="avg_temp", value=str(avg_temp)))
        diagnostics_status.values.append(KeyValue(key="avg_cpu", value=str(avg_cpu)))
        diagnostics_status.values.append(KeyValue(key="avg_gpu", value=str(avg_gpu)))
        diagnostics_status.values.append(KeyValue(key="mavros_init", value=str(self.mavros_init)))
        diagnostics_status.values.append(KeyValue(key="camera_init", value=str(self.camera_init)))

        diagnostics_array.status.append(diagnostics_status)
        self.diagnostics_pub.publish(diagnostics_array)

    def main_loop(self):
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            r.sleep()
            try:
                mavros_freq = self.mavros_hz.get_hz()
                mavros_freq = mavros_freq[0]
                if(mavros_freq >= 40 and self.mavros_init==False):
                    self.mavros_init = True
                    self.publish_notification("low level ready")
                if mavros_freq < 40:
                    self.mavros_init = False
                    os.system(self.mavros_action)
                camera_freq = self.camera_hz.get_hz()
                camera_freq = camera_freq[0]
                if(camera_freq > 20 and self.camera_init==False):
                    self.camera_init = True
                    self.publish_notification("camera ready")
                    print("camera READY")
            except:
                pass
            self.publish_diagnostics()

if __name__ == '__main__':
    rospy.init_node("HAL_9000", anonymous=True)
    hal_obj = hal("/root/catkin_ws/src/hound_core/config/HAL.yaml")




