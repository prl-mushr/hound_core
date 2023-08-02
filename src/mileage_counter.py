import rosbag
import os
from vesc_msgs.msg import VescStateStamped
import numpy as np

bagdir = '/root/catkin_ws/src/new_bags'
files = os.listdir(bagdir)
files.sort(key = lambda x: os.path.getmtime(bagdir+'/'+x))
mileage = 0
speed = []
lat_acc = []
for i in range(len(files)):
    source = bagdir + '/' + files[i]
    bag = rosbag.Bag(source, 'r')
    print("processed: {} percent".format(100*i/len(files)))
    for topic, msg, t in bag.read_messages(topics=['/sensors/core', '/mavros/imu/data_raw']):
        if topic == '/sensors/core':
            mileage += (msg.state.speed*0.02)/3166
            speed.append(msg.state.speed/3166)
        elif topic == '/mavros/imu/data_raw':
            lat_acc.append(abs(msg.linear_acceleration.y))

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