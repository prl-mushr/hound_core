import numpy as np
from scipy import signal

states = np.load("states.npy")

A = states[:, 9:12]
G = states[:, 12:15]
RPY = states[:, 3:6]
V = states[:, 6:9]
G = states[:, 12:15]
speeds = states[:, 15]

print("hours in states: ", states.shape[0] * 0.02 / 3600)
roll_indices = np.where((np.fabs(RPY[:, 0]) > 1.0) & (speeds > 0.5))[0]
roll_starts = np.where(np.diff(roll_indices) > 50)
roll_indices = roll_indices[roll_starts]
mode = states[roll_indices, 17]
print("mode: ", mode)
roll_indices -= 10  ## go back 0.2 seconds.
print("number of rollovers: ", len(roll_indices))
roll_vert_acc = np.fabs(A[roll_indices, 2])
print("vertical acceleration during rollover", roll_vert_acc)
print("avg_vert_acc_auto: ", np.mean(roll_vert_acc[np.where(mode)]))
print("avg_vert_acc_manual: ", np.mean(roll_vert_acc[np.where(mode == 0)]))

# remove data points where the car rolled over to avoid counting lateral acceleration from rolls
states = np.delete(states, roll_indices, 0)
auto = np.where(states[:, 17])
manl = np.where(states[:, 17] == 0)
auto_dist = np.sum(speeds[auto] * 0.02)
manl_dist = np.sum(speeds[manl] * 0.02)
print("auto dist:", auto_dist * 1e-3, " km")

order = 2  # Filter order
cutoff_freq = 0.04  # 10hz. utoff frequency as a fraction of Nyquist frequency
b, a = signal.butter(order, cutoff_freq, btype="low", analog=False)
lat_acc = np.fabs(signal.lfilter(b, a, A[:, 1] - 9))
body_vel = np.fabs(signal.lfilter(b, a, V[:, 0]))
body_vel[np.where(body_vel > speeds)] = 0

auto_lat_sort = np.sort(lat_acc[auto])
manl_lat_sort = np.sort(lat_acc[manl])
auto_spd_sort = np.sort(body_vel[auto])
manl_spd_sort = np.sort(body_vel[manl])

auto_lat_max = auto_lat_sort[-500]
manl_lat_max = manl_lat_sort[-500]
auto_spd_max = auto_spd_sort[-500]
manl_spd_max = manl_spd_sort[-500]


print("peak lateral acceleration in auto mode: ", auto_lat_max)
print("peak lateral acceleration in manual mode: ", manl_lat_max)
print("peak speed in auto mode: ", auto_spd_max)
print("peak speed in manual mode: ", manl_spd_max)
print(
    "mean/std lateral acceleration: ", np.fabs(A[:, 1]).mean(), np.fabs(A[:, 1]).std()
)
print("mean/std speed: ", V[:, 0].mean(), V[:, 0].std())
