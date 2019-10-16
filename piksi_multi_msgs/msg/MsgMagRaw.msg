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

# Mag
# MSG_MAG_RAW
#
# Raw magnetometer data
#
# Raw data from the magnetometer.

# Milliseconds since start of GPS week. If the high bit is set, the
# time is unknown or invalid.
# [ms]
uint32 tow
# Milliseconds since start of GPS week, fractional part
# [ms / 256]
uint8 tow_f
# Magnetic field in the body frame X axis
# [microteslas]
int16 mag_x
# Magnetic field in the body frame Y axis
# [microteslas]
int16 mag_y
# Magnetic field in the body frame Z axis
# [microteslas]
int16 mag_z