#!/usr/bin/env python3
# Import ROS libraries and messages
import os
os.system('usbreset "ChibiOS/RT Virtual COM Port"') ## this gets executed with 0 delay
import rospy
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rostopic import ROSTopicHz
import yaml
from mavros_msgs.msg import PlayTuneV2, RCIn, GPSRAW, State
import subprocess, shlex, psutil
from vesc_msgs.msg import VescStateStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import tf
import numpy as np
import time
import platform

class hal():
    def __init__(self, config_file):
        with open(config_file) as f:
            config = yaml.safe_load(f)
            print("configs found")
        if(config == None):
            print("no config found, make sure the config file path is correct")
            exit()

        self.on_jetson = platform.machine() == 'aarch64'
        self.initial_pos = None

        self.mavros_config = config["mavros"]
        self.camera_config = config["camera"]
        self.lidar_config = config["lidar"]
        self.vesc_config = config["vesc"]
        self.dawg_config = config["dawg"]
        self.mavros_hz = ROSTopicHz(3)

        self.camera_hz = ROSTopicHz(3)

        sub_mavros = rospy.Subscriber(self.mavros_config["monitor_topic"], rospy.AnyMsg, self.mavros_hz.callback_hz)
        sub_camera = rospy.Subscriber(self.camera_config["monitor_topic"], rospy.AnyMsg, self.camera_hz.callback_hz)

        sub_channel = rospy.Subscriber(self.mavros_config["channel_topic"], RCIn, self.channel_cb, queue_size=2)
        sub_voltage = rospy.Subscriber(self.vesc_config["topic"], VescStateStamped, self.voltage_cb, queue_size = 1)
        sub_gps = rospy.Subscriber(self.mavros_config["gps_topic"], GPSRAW, self.GPS_cb, queue_size = 1)
        pose_sub = rospy.Subscriber(self.mavros_config["pose_topic"], PoseStamped, self.pose_cb, queue_size=1)
        scan_sub = rospy.Subscriber(self.lidar_config["scan_topic"], LaserScan, self.scan_cb)
        mavros_status_sub = rospy.Subscriber(self.mavros_config["state_topic"], State, self.mavros_status_cb, queue_size=10)
        self.last_map_clear = time.time()
        
        self.lp = lg.LaserProjection()
        self.pc_pub = rospy.Publisher(self.lidar_config["pc_topic"], PointCloud2, queue_size=1)
        self.last_pose_time = None

        self.notification_pub = rospy.Publisher(self.mavros_config["notification_topic"], PlayTuneV2, queue_size =10)
        
        self.bagdir = config["bagdir"]
        record_command_file = config["record_command_file"]
        with open(record_command_file, 'r') as f:
            self.record_command = f.read()
        
        self.mavros_init = False
        self.camera_init = False
        self.dawg_init   = False
        
        self.recording_state = False
        self.rosbag_proc = None
        self.dawg_proc   = None
        self.GPS_status = False

        self.diagnostics_pub = rospy.Publisher(config["diagnostics_topic"], DiagnosticArray, queue_size=2)
        
        time.sleep(15)
        os.system(self.mavros_config["failure_action"]) ## first attempt at getting mavros to run @50 Hz
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
        self.command = 'rosbag record --split --duration=5m -O ' + self.bagdir + 'temp ' + self.record_command
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
        dawg_files = list(filter(lambda file : file[ : 4] in ("dawg", "temp"), files))
        dawg_files.sort(key = lambda x: os.path.getmtime(self.bagdir+'/'+x))
        for i in range(len(dawg_files)):
            source = self.bagdir + '/' + dawg_files[i]
            dest = self.bagdir + '/' + 'dawg_' + str(i) + '.bag'
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

    def start_dawg(self):
        cmd = shlex.split(self.dawg_config["dawg_launch"])
        self.dawg_proc = subprocess.Popen(cmd)

    def stop_dawg(self):
        self.dawg_proc.send_signal(subprocess.signal.SIGINT)
    
    def channel_cb(self, rc):
        try:
            # Button A for recording
            if (rc.channels[6] > 1900):
                if not self.recording_state:
                    self.recording_state = True
                    print("start recording..")
                    self.start_recording()
            else:
                if self.recording_state:
                    self.recording_state = False
                    print("stop recording")
                    self.stop_recording()

            if (rc.channels[7] > 1900):
                if not self.dawg_init:
                    self.dawg_init = True
                    print("Dawg initiated!")
                    self.start_dawg()
            else:
                if self.dawg_init:
                    self.dawg_init = False
                    print("Dawg down!")
                    self.stop_dawg()

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
        
        # if self.initial_pos is None:
        #     self.initial_pos = pos
        # pos.x -= self.initial_pos.x
        # pos.y -= self.initial_pos.y
        # pos.z -= self.initial_pos.z

        br.sendTransform((pos.x, pos.y, pos.z),
                         (rot.x, rot.y, rot.z, rot.w),
                         msg.header.stamp,
                         self.mavros_config["frame"],
                         "map")
        br.sendTransform(tuple(self.camera_config["pos"]),
                         tuple(self.camera_config["rot"]),
                         msg.header.stamp,
                         self.camera_config["depth_frame"],
                         self.mavros_config["frame"])

        br.sendTransform((0, 0, 0),
                         (0, 0, 0, 1),
                         msg.header.stamp,
                         "odom",
                         "map")

        br.sendTransform((0, 0, 0),
                         (-0.5, 0.5, -0.5, 0.5),
                         msg.header.stamp,
                         self.camera_config["depth_optical_frame"],
                         self.camera_config["depth_frame"])

        br.sendTransform(tuple(self.lidar_config["pos"]),
                         tuple(self.lidar_config["rot"]),
                         msg.header.stamp,
                         self.lidar_config["frame"],
                         self.mavros_config["frame"])
        self.last_pose_time = msg.header.stamp

    def mavros_status_cb(self, msg):
        armed = msg.armed
        if not armed and time.time() - self.last_map_clear > 1.0:
            os.system('rosservice call /elevation_mapping/clear_map') ## worst (but easiest?) way to do a rosservice call when you don't care about returns
            self.last_map_clear = time.time()

    def publish_diagnostics(self):
        if not self.on_jetson:
            return
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
                if(mavros_freq >= 0.8*self.mavros_config["fps"] and self.mavros_init==False):
                    self.mavros_init = True
                    self.publish_notification("low level ready")
                if mavros_freq < 0.8*self.mavros_config["fps"]:
                    self.mavros_init = False
                    os.system(self.mavros_config["failure_action"])
                camera_freq = self.camera_hz.get_hz()
                camera_freq = camera_freq[0]
                if(camera_freq > 0.8*self.camera_config["fps"] and self.camera_init==False):
                    self.camera_init = True
                    self.publish_notification("camera ready")
                    print("camera READY")
            except:
                pass
            self.publish_diagnostics()

if __name__ == '__main__':
    rospy.init_node("HAL_9000", anonymous=True)
    hal_obj = hal("/root/catkin_ws/src/dawg_core/config/HAL.yaml")