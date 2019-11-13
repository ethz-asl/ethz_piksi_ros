#!/usr/bin/env python

import pandas as pd
import rosbag
import numpy as np
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt

def loadMessages(file, piksi_topic, external_imu_topic):
    bag = rosbag.Bag(file)

    omega_piksi = np.zeros(bag.get_message_count(piksi_topic))
    times_piksi = np.zeros(len(omega_piksi))

    omega_external = np.zeros(bag.get_message_count(external_imu_topic))
    times_external = np.zeros(len(omega_external))

    idx_piksi = 0
    idx_external = 0
    for topic, msg, t in bag.read_messages(topics=[piksi_topic, external_imu_topic]):
        omega = np.linalg.norm([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if topic == piksi_topic:
            omega_piksi[idx_piksi] = omega
            times_piksi[idx_piksi] = msg.header.stamp.to_sec()
            idx_piksi += 1
        elif topic == external_imu_topic:
            omega_external[idx_external] = omega
            times_external[idx_external] = msg.header.stamp.to_sec()
            idx_external += 1

    print("Loaded %u messages from %s" %(len(omega_piksi), piksi_topic))
    print("Loaded %u messages from %s" %(len(omega_external), external_imu_topic))

    return omega_piksi, times_piksi, omega_external, times_external

def getSamplingPeriod(rng):
    return (rng[-1] - rng[0]) / len(rng)

# Returns the equally spaced time index series between the union of border times according to the faster frequency.
def getEquallySpacedTimeSeries(rng1, rng2):
    dt = min(getSamplingPeriod(rng1), getSamplingPeriod(rng2))
    t_min = max(rng1.min(), rng2.min())
    t_max = min(rng1.max(), rng2.max())
    return pd.DatetimeIndex(start=t_min, end=t_max, freq=dt), dt

def computeTimeOffset(omega_piksi, times_piksi, omega_external, times_external):
    # Correlation.
    print("Computing time offset.")

    # Find equally spaced time index according to faster series.
    rng_piksi = pd.to_datetime(times_piksi, unit='s')
    rng_external = pd.to_datetime(times_external, unit='s')
    rng, dt = getEquallySpacedTimeSeries(rng_piksi, rng_external)

    # Create time series and reindex.
    ts_piksi = pd.Series(omega_piksi, rng_piksi)
    ts_external = pd.Series(omega_external, rng_external)

    ts_piksi = ts_piksi.reindex(rng.union(rng_piksi)).interpolate(method='time').reindex(rng)
    ts_external = ts_external.reindex(rng.union(rng_external)).interpolate(method='time').reindex(rng)

    # Find cross correlation.
    conv = np.correlate(ts_piksi.values, ts_external.values, 'full')

    # Deviation of the maximum convolution from the middle of the convolution vector
    dn = len(conv) / 2 - np.argmax(conv)
    t_off = dt.total_seconds() * dn
    print("Time offset t_off from piksi to external IMU t_external_aligned = t_external - t_off: %f" % t_off)

    return t_off

def plotCorrelation(omega_piksi, times_piksi, omega_external, times_external, t_off):
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, sharey=True)

    ax1.plot(times_piksi, omega_piksi, 'ro', label='Piksi', )
    ax1.plot(times_external, omega_external, color='g', label='External')
    ax1.set_ylabel('Angular Velocity [rad/s]')
    ax1.set_title('Uncorrelated Angular Velocity')
    ax1.legend()

    times_external_aligned = times_external - t_off
    ax2.plot(times_piksi, omega_piksi, 'ro', label='Piksi')
    ax2.plot(times_external_aligned, omega_external, color='g', label='Adis')
    ax2.set_title('Correlated Angular Velocity (t_off=%.3fs)' % (t_off))
    ax2.set_ylabel('Angular Velocity [rad/s]')
    ax2.set_xlabel('Time [s]')

    plt.show()
