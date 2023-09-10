import rosbag
import os
from vesc_msgs.msg import VescStateStamped
import tf
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy import signal

bagdir_list = ['new_bags', 'new_bags_2', 'new_bags_d455', 'new_bags_ROI' ,'new_bags_bmx']
base = '/root/catkin_ws/src/'

for bagdir_name in bagdir_list:
    bagdir = base + bagdir_name
    files = os.listdir(bagdir)
    files.sort(key = lambda x: os.path.getmtime(bagdir+'/'+x))

    try:
        mileage = np.load('mileage.npy')
        speed = list(np.load('speeds.npy'))
        lat_acc = list(np.load('lat_acc.npy'))
        states = list(np.load('states.npy'))
        statistics = list(np.load('statistics.npy'))

    except:
        mileage = 0
        speed = []
        lat_acc = []
        states = []

    for i in range(len(files)):
        source = bagdir + '/' + files[i]
        bag = rosbag.Bag(source, 'r')
        print("processed: {} percent".format(100*i/len(files)))
        state = np.zeros(18)
        for topic, msg, t in bag.read_messages(topics=['/sensors/core', '/mavros/imu/data_raw', '/mavros/local_position/odom', '/mavros/manual_control/send', '/mavros/rc/in']):
            if topic == '/sensors/core':
                mileage += (msg.state.speed*0.02)/3166
                speed.append(msg.state.speed/3166)
                state[15] = msg.state.speed/3166
            elif topic == '/mavros/imu/data_raw':
                lat_acc.append(abs(msg.linear_acceleration.y))
                state[9:12] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            elif topic == '/mavros/local_position/odom':
                state[0:3] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
                rpy = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                state[3:6] = [rpy[0], rpy[1], rpy[2]]
                state[6:9] = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
                state[12:15] = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
            elif topic == '/mavros/manual_control/send':
                state[16] = -msg.y/1000.0
            elif topic == '/mavros/rc/in':
                mode_switch = msg.channels[4]
                auto = False
                if(mode_switch > 1800):
                    auto = True
                state[17] = auto
            if state[:15].all() != 0:
                states.append(state)
                state = np.zeros(18)

order = 2  # Filter order
cutoff_freq = 0.04  # Cutoff frequency as a fraction of Nyquist frequency
b, a = signal.butter(order, cutoff_freq, btype='low', analog=False)
lat_acc = np.fabs(signal.lfilter(b, a, lat_acc))

speed = np.array(speed)
lat_acc = np.array(lat_acc)
mean_speed = np.mean(speed)
mean_lat_acc = np.mean(lat_acc)
percentile_speed = abs(np.percentile(speed, 97.5) - np.percentile(speed, 2.5))/2
percentile_acc = abs(np.percentile(lat_acc, 97.5) - np.percentile(lat_acc, 2.5))/2
max_speed = np.max(speed)
max_acc = np.max(lat_acc)
print(mean_speed, percentile_speed, max_speed)
print(mean_lat_acc, percentile_acc, max_acc)
print(mileage*1e-3)
np.save('statistics.npy',np.array([mileage, mean_speed, percentile_speed, mean_lat_acc, percentile_acc]))
np.save('speeds.npy', speed)
np.save('lat_acc.npy', lat_acc)
np.save('states.npy', states)
np.save('mileage.npy', mileage)