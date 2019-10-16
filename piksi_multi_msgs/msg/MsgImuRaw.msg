# Copyright (C) 2018 Swift Navigation Inc.
# Contact: Swift Navigation <dev@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Imu
# MSG_IMU_RAW
#
# Raw IMU data
#
# Raw data from the Inertial Measurement Unit, containing accelerometer and
# gyroscope readings. The sense of the measurements are to be aligned with 
# the indications on the device itself. Measurement units, which are specific to the
# device hardware and settings, are communicated via the MSG_IMU_AUX message.

# Milliseconds since start of GPS week. If the high bit is set, the
# time is unknown or invalid.
# [ms]
uint32 tow
# Milliseconds since start of GPS week, fractional part
# [ms / 256]
uint8 tow_f
# Acceleration in the IMU frame X axis
# []
int16 acc_x
# Acceleration in the IMU frame Y axis
# []
int16 acc_y
# Acceleration in the IMU frame Z axis
# []
int16 acc_z
# Angular rate around IMU frame X axis
# []
int16 gyr_x
# Angular rate around IMU frame Y axis
# []
int16 gyr_y
# Angular rate around IMU frame Z axis
# []
int16 gyr_z