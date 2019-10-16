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
# MSG_ANGULAR_RATE
#
# Vehicle Body Frame instantaneous angular rates
#
# This message reports the orientation rates in the vehicle body frame. 
# The values represent the measurements a strapped down gyroscope would 
# make and are not equivalent to the time derivative of the Euler angles.
# The orientation and origin of the user frame is specified via device settings.
# By convention, the vehicle x-axis is expected to be aligned with the forward
# direction, while the vehicle y-axis is expected to be aligned with the right
# direction, and the vehicle z-axis should be aligned with the down direction.
# This message will only be available in future INS versions of Swift Products 
# and is not produced by Piksi Multi or Duro.

# GPS Time of Week
# [ms]
uint32 tow
# angular rate about x axis
# [microdegrees/s]
int32 x
# angular rate about y axis
# [microdegrees/s]
int32 y
# angular rate about z axis
# [microdegrees/s]
int32 z
# Status flags
# []
uint8 flags