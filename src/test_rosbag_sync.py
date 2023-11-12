#!/usr/bin/env python3
import rosbag
from std_msgs.msg import Int32, String  # Replace with your message types
import rospy

bag = rosbag.Bag('/root/catkin_ws/src/bags/hound_19.bag')

# Create dictionaries to store messages from different topics
topic1_msgs = {}
topic2_msgs = {}


for topic, msg, t in bag.read_messages(topics=['/mavros/imu/data_raw', '/mavros/local_position/odom']):
    if topic == '/mavros/imu/data_raw':
        topic1_msgs[t] = msg
        # print("topic_1_t", t.to_sec())
    elif topic == '/mavros/local_position/odom':
        topic2_msgs[t] = msg
        # print("topic_2_t", t.to_sec())

# Synchronize messages based on their timestamps
counter = 0
min_length = min(len(topic1_msgs), len(topic2_msgs))
print(min_length)
tolerance = 0.00
while counter < min_length*0.9:
    tolerance += 5e-4
    time_window = rospy.Duration(tolerance)  # Adjust this as needed
    for t1, msg1 in topic1_msgs.items():
        for t2, msg2 in topic2_msgs.items():
            time_diff = abs((t1 - t2).to_sec())
            if time_diff <= time_window.to_sec():
                counter += 1
    print(counter)
bag.close()
