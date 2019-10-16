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
# MSG_IMU_AUX
#
# Auxiliary IMU data
#
# Auxiliary data specific to a particular IMU. The `imu_type` field will
# always be consistent but the rest of the payload is device specific and
# depends on the value of `imu_type`.

# IMU type
# []
uint8 imu_type
# Raw IMU temperature
# []
int16 temp
# IMU configuration
# []
uint8 imu_conf