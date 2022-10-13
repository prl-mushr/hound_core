#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
import time
from diagnostic_msgs.msg import DiagnosticArray
from rostopic import ROSTopicHz
import yaml

rospy.init_node("diagnostics_listener", anonymous=True)

class hal():
    def __init__(self, config_file):
        with open(config_file) as f:
            config = yaml.safe_load(f)
            print("sensor configs found")
        if(config == None):
            print("no config found, make sure the config file path is correct")
            exit()
        self.camera_launch_file = config["camera_launch_file"]
        self.camera_frequency = config["camera_frequency"] * 0.8
        self.camera_topic = config["camera_topic"]
        self.camera_action = config["camera_action"]
        self.camera_hz = ROSTopicHz(3)  # get frequency over a 3 second period
        
        self.mavros_launch_file = config["mavros_launch_file"]
        self.mavros_frequency = config["mavros_frequency"] * 0.8
        self.mavros_topic = config["mavros_topic"]
        self.mavros_action = config["mavros_action"]
        self.mavros_hz = ROSTopicHz(3)
        
        self.lidar_launch_file = config["lidar_launch_file"]
        self.lidar_frequency = config["lidar_frequency"] * 0.8
        self.lidar_action = config["lidar_action"]
        self.lidar_topic = config["lidar_topic"]
        self.lidar_hz = ROSTopicHz(3)
        
        self.vesc_launch_file = config["vesc_launch_file"]
        self.vesc_frequency = config["vesc_frequency"] * 0.8
        self.vesc_action = config["vesc_action"]
        self.vesc_topic = config["vesc_topic"]
        self.vesc_hz = ROSTopicHz(3)

        #self.diagnostics_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostic_cb, queue_size = 10)
        sub = rospy.Subscriber(self.camera_topic, rospy.AnyMsg, self.camera_hz.callback_hz)
        sub = rospy.Subscriber(self.mavros_topic, rospy.AnyMsg, self.mavros_hz.callback_hz)
        sub = rospy.Subscriber(self.lidar_topic, rospy.AnyMsg, self.lidar_hz.callback_hz)
        sub = rospy.Subscriber(self.vesc_topic, rospy.AnyMsg, self.vesc_hz.callback_hz)

        self.main_loop()

    def diagnostic_cb(self, msg):
        print("============================")
        info = msg.status[0]
        print("status:", info.level)
        print("name: ", info.name)
        print("value:", info.values[3].value)

    def main_loop(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        mavros_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.mavros_launch_file])
        mavros_launch.start()
        time.sleep(1)

        camera_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.camera_launch_file])
        camera_launch.start()
        time.sleep(1)

        lidar_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.lidar_launch_file])
        lidar_launch.start()
        time.sleep(1)

        vesc_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.vesc_launch_file])
        vesc_launch.start()
        
        time.sleep(10)  # give all the nodes enough time to boot up. Make sure none of these sensors depend on each other to avoid race conditions

        while not rospy.is_shutdown():
            if(self.camera_hz.get_hz() < self.camera_frequency):
                try:
                    camera_launch.shutdown()
                except:
                    pass
                time.sleep(1)
                camera_launch.start()
                time.sleep(5)

            if(self.mavros_hz.get_hz() < self.mavros_frequency):
                try:
                    mavros_launch.shutdown()
                except:
                    pass
                time.sleep(1)
                mavros_launch.start()
                time.sleep(5)

            if(self.lidar_hz.get_hz() < self.lidar_frequency):
                try:
                    lidar_launch.shutdown()
                except:
                    pass
                time.sleep(1)
                lidar_launch.start()
                time.sleep(5)

            if(self.vesc_hz.get_hz() < self.vesc_frequency):
                try:
                    vesc_launch.shutdown()
                except:
                    pass
                time.sleep(1)
                vesc_launch.start()
                time.sleep(5)

if __name__ == '__main__':
    hal_obj = hal("/root/catkin_ws/src/hound_core/config/HAL.yaml")




