#!/usr/bin/env python

import pandas as pd
import rosbag
import numpy as np
from sensor_msgs.msg import Imu
#from cuckoo_time_translator import DeviceTimestamp
import matplotlib.pyplot as plt

def getSamplingPeriod(rng):
    return (rng[-1] - rng[0]) / len(rng)

# Returns the equally spaced time index series between the union of border times according to the faster frequency.
def getEquallySpacedTimeSeries(rng1, rng2):
    dt = min(getSamplingPeriod(rng1), getSamplingPeriod(rng2))
    t_min = max(rng1.min(), rng2.min())
    t_max = min(rng1.max(), rng2.max())
    return pd.DatetimeIndex(start=t_min, end=t_max, freq=dt), dt

file = "/home/rik/data/2019_05_21_piksi_adis/2019-05-22-13-09-35.bag"
piksi_topic = "/moa/piksi/imu"
adis_topic = "/moa/imu/data"
device_time_topic = "/moa/device_time"

# Read postprocessed ENU positions.
bag = rosbag.Bag(file)

omega_piksi = np.zeros(bag.get_message_count(piksi_topic))
times_piksi = np.zeros(len(omega_piksi))

omega_adis = np.zeros(bag.get_message_count(adis_topic))
times_adis = np.zeros(len(omega_adis))

adis_device_time = {}

idx_piksi = 0
idx_adis = 0
for topic, msg, t in bag.read_messages(topics=[piksi_topic, adis_topic, device_time_topic]):
    omega = 0.0
    if msg._type == 'sensor_msgs/Imu':
        omega = np.linalg.norm([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    if topic == piksi_topic:
        omega_piksi[idx_piksi] = omega
        times_piksi[idx_piksi] = msg.header.stamp.to_sec()
        idx_piksi += 1
    elif topic == adis_topic:
        omega_adis[idx_adis] = omega
        times_adis[idx_adis] = msg.header.stamp.to_sec()
        idx_adis += 1
    elif topic == device_time_topic:
        adis_device_time[msg.header.stamp.to_sec() - 0.024] = msg.event_stamp

print("Loaded %u messages from %s" %(len(omega_piksi), piksi_topic))
print("Loaded %u messages from %s" %(len(omega_adis), adis_topic))

# Correlation.
print("Computing time offset.")
# Find equally spaced time index according to faster series.
rng_piksi = pd.to_datetime(times_piksi, unit='s')
rng_adis = pd.to_datetime(times_adis, unit='s')
rng, dt = getEquallySpacedTimeSeries(rng_piksi, rng_adis)

# Create time series and reindex.
ts_piksi = pd.Series(omega_piksi, rng_piksi)
ts_adis = pd.Series(omega_adis, rng_adis)

ts_piksi = ts_piksi.reindex(rng.union(rng_piksi)).interpolate(method='time').reindex(rng)
ts_adis = ts_adis.reindex(rng.union(rng_adis)).interpolate(method='time').reindex(rng)

# Find cross correlation.
conv = np.correlate(ts_piksi.values, ts_adis.values, 'full')

# Deviation of the maximum convolution from the middle of the convolution vector
dn = len(conv) / 2 - np.argmax(conv)
t_off = dt.total_seconds() * dn
print("Time offset from piksi to adis t_adis_aligned = t_adis - t_off: %f" % t_off)

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, sharey=True)

ax1.plot(times_piksi, omega_piksi, 'ro', label='Piksi', )
ax1.plot(times_adis, omega_adis, color='g', label='Adis')
ax1.set_ylabel('Angular Velocity [rad/s]')
ax1.set_title('Uncorrelated Angular Velocity')
ax1.legend()

times_adis_aligned = times_adis - t_off
ax2.plot(times_piksi, omega_piksi, 'ro', label='Piksi')
ax2.plot(times_adis_aligned, omega_adis, color='g', label='Adis')
ax2.set_title('Correlated Angular Velocity (t_off=%.3fs)' % (t_off))
ax2.set_ylabel('Angular Velocity [rad/s]')
ax2.set_xlabel('Time [s]')

## Plot with device time
#adis_device_time_new = { time_adis: adis_device_time[time_adis] for time_adis in times_adis }
#adis_device_time = adis_device_time_new.values()
#adis_device_time.sort()
#
#fig, (ax) = plt.subplots(1, 1)
#ax.plot(adis_device_time, omega_adis, color='r', label='Adis')
#ax.set_ylabel('Angular Velocity [rad/s]')
#ax.set_xlabel('Device Time [us]')
#ax.legend()
#ax.grid()

plt.show()
