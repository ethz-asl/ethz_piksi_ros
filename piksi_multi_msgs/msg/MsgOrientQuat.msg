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

# Orientation
# MSG_ORIENT_QUAT
#
# Quaternion 4 component vector
#
# This message reports the quaternion vector describing the vehicle body frame's orientation
# with respect to a local-level NED frame. The components of the vector should sum to a unit
# vector assuming that the LSB of each component as a value of 2^-31. This message will only
# be available in future INS versions of Swift Products and is not produced by Piksi Multi 
# or Duro.

# GPS Time of Week
# [ms]
uint32 tow
# Real component
# [2^-31]
int32 w
# 1st imaginary component
# [2^-31]
int32 x
# 2nd imaginary component
# [2^-31]
int32 y
# 3rd imaginary component
# [2^-31]
int32 z
# Estimated standard deviation of w
# [N/A]
float32 w_accuracy
# Estimated standard deviation of x
# [N/A]
float32 x_accuracy
# Estimated standard deviation of y
# [N/A]
float32 y_accuracy
# Estimated standard deviation of z
# [N/A]
float32 z_accuracy
# Status flags
# []
uint8 flags