#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
import time
from diagnostic_msgs.msg import DiagnosticArray
from rostopic import ROSTopicHz
import yaml
from mavros_msgs.msg import PlayTuneV2, RCIn, GPSRAW
import subprocess, shlex, psutil
from vesc_msgs.msg import VescStateStamped
import os
rospy.init_node("diagnostics_listener", anonymous=True)

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

        sub_mavros = rospy.Subscriber(self.mavros_topic, rospy.AnyMsg, self.mavros_hz.callback_hz)
        sub_channel = rospy.Subscriber("mavros/rc/in", RCIn, self.channel_cb, queue_size=2)
        sub_voltage = rospy.Subscriber("sensors/core", VescStateStamped, self.voltage_cb, queue_size = 1)
        sub_gps = rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, self.GPS_cb, queue_size = 1)
        self.notification_pub = rospy.Publisher("/mavros/play_tune", PlayTuneV2, queue_size =10)
        
        
        self.bagdir = '/root/catkin_ws/src/bags/'
        self.mavros_init = False
        self.recording_state = False
        self.rosbag_proc = None
        self.GPS_status = False
        time.sleep(15)
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
        self.notification_pub.publish(playtune)
    
    def start_recording(self):
        self.command = 'rosbag record -O ' + self.bagdir + 'hound -a -x "(.*)/(.*)/(.*)/theora" -x "(.*)/(.*)/(.*)/theora/(.*)" -x "(.*)/(.*)/(.*)/theora" ' # -e "/mavros/(.*)" /sensors/core -x /mavros/gps_rtk/(.*)'
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
            stick = rc.channels[1]
            if(self.recording_state == False and stick > 1800):
                print("start recording")
                self.start_recording()
                self.recording_state = True
            elif(self.recording_state == True and stick < 1300):
                print("stop recording")
                self.stop_recording()
                self.recording_state = False
        except:
            pass
        
    def main_loop(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()
            try:
                mavros_freq = self.mavros_hz.get_hz()
                mavros_freq = mavros_freq[0]
                if(mavros_freq >= 40 and self.mavros_init==False):
                    self.mavros_init = True
                    self.publish_notification("low level ready")
                # print(mavros_freq)
            except:
                pass

if __name__ == '__main__':
    hal_obj = hal("/root/catkin_ws/src/hound_core/config/HAL.yaml")




