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
# MSG_VEL_BODY
#
# Velocity in User Frame
#
# This message reports the velocity in the Vehicle Body Frame. By convention,
# the x-axis should point out the nose of the vehicle and represent the forward
# direction, while as the y-axis should point out the right hand side of the vehicle.
# Since this is a right handed system, z should point out the bottom of the vehicle.
# The orientation and origin of the Vehicle Body Frame are specified via the device settings.
# The full GPS time is given by the preceding MSG_GPS_TIME with the
# matching time-of-week (tow). This message is only produced by inertial versions of Swift
# products and is not available from Piksi Multi or Duro.

# GPS Time of Week
# [ms]
uint32 tow
# Velocity in x direction
# [mm/s]
int32 x
# Velocity in y direction
# [mm/s]
int32 y
# Velocity in z direction
# [mm/s]
int32 z
# Estimated variance of x
# [m^2]
float32 cov_x_x
# Covariance of x and y
# [m^2]
float32 cov_x_y
# Covariance of x and z
# [m^2]
float32 cov_x_z
# Estimated variance of y
# [m^2]
float32 cov_y_y
# Covariance of y and z
# [m^2]
float32 cov_y_z
# Estimated variance of z
# [m^2]
float32 cov_z_z
# Number of satellites used in solution
# []
uint8 n_sats
# Status flags
# []
uint8 flags