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
# MSG_VEL_NED_COV
#
# Velocity in NED
#
# This message reports the velocity in local North East Down (NED)
# coordinates. The NED coordinate system is defined as the local WGS84
# tangent plane centered at the current position. The full GPS time is
# given by the preceding MSG_GPS_TIME with the matching time-of-week (tow).
# This message is similar to the MSG_VEL_NED, but it includes the upper triangular
# portion of the 3x3 covariance matrix.

# GPS Time of Week
# [ms]
uint32 tow
# Velocity North coordinate
# [mm/s]
int32 n
# Velocity East coordinate
# [mm/s]
int32 e
# Velocity Down coordinate
# [mm/s]
int32 d
# Estimated variance of northward measurement
# [m^2]
float32 cov_n_n
# Covariance of northward and eastward measurement
# [m^2]
float32 cov_n_e
# Covariance of northward and downward measurement
# [m^2]
float32 cov_n_d
# Estimated variance of eastward measurement
# [m^2]
float32 cov_e_e
# Covariance of eastward and downward measurement
# [m^2]
float32 cov_e_d
# Estimated variance of downward measurement
# [m^2]
float32 cov_d_d
# Number of satellites used in solution
# []
uint8 n_sats
# Status flags
# []
uint8 flags