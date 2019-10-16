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
# MSG_POS_LLH_COV
#
# Geodetic Position
#
# This position solution message reports the absolute geodetic
# coordinates and the status (single point vs pseudo-absolute RTK)
# of the position solution as well as the upper triangle of the 3x3
# covariance matrix.  The position information and Fix Mode flags should
# follow the MSG_POS_LLH message.  Since the covariance matrix is computed
# in the local-level North, East, Down frame, the covariance terms follow
# with that convention. Thus, covariances are reported against the "downward"
# measurement and care should be taken with the sign convention.

# GPS Time of Week
# [ms]
uint32 tow
# Latitude
# [deg]
float64 lat
# Longitude
# [deg]
float64 lon
# Height above WGS84 ellipsoid
# [m]
float64 height
# Estimated variance of northing
# [m^2]
float32 cov_n_n
# Covariance of northing and easting
# [m^2]
float32 cov_n_e
# Covariance of northing and downward measurement
# [m^2]
float32 cov_n_d
# Estimated variance of easting
# [m^2]
float32 cov_e_e
# Covariance of easting and downward measurement
# [m^2]
float32 cov_e_d
# Estimated variance of downward measurement
# [m^2]
float32 cov_d_d
# Number of satellites used in solution.
# []
uint8 n_sats
# Status flags
# []
uint8 flags