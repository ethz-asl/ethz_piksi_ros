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

# Navigation
# MSG_VEL_ECEF_COV
#
# Velocity in ECEF
#
# This message reports the velocity in Earth Centered Earth Fixed
# (ECEF) coordinates. The full GPS time is given by the preceding
# MSG_GPS_TIME with the matching time-of-week (tow).

# GPS Time of Week
# [ms]
uint32 tow
# Velocity ECEF X coordinate
# [mm/s]
int32 x
# Velocity ECEF Y coordinate
# [mm/s]
int32 y
# Velocity ECEF Z coordinate
# [mm/s]
int32 z
# Estimated variance of x
# [m^2/s^2]
float32 cov_x_x
# Estimated covariance of x and y
# [m^2/s^2]
float32 cov_x_y
# Estimated covariance of x and z
# [m^2/s^2]
float32 cov_x_z
# Estimated variance of y
# [m^2/s^2]
float32 cov_y_y
# Estimated covariance of y and z
# [m^2/s^2]
float32 cov_y_z
# Estimated variance of z
# [m^2/s^2]
float32 cov_z_z
# Number of satellites used in solution
# []
uint8 n_sats
# Status flags
# []
uint8 flags