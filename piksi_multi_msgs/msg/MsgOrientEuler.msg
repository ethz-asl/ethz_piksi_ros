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
# MSG_ORIENT_EULER
#
# Euler angles
#
# This message reports the yaw, pitch, and roll angles of the vehicle body frame.
# The rotations should applied intrinsically in the order yaw, pitch, and roll 
# in order to rotate the from a frame aligned with the local-level NED frame 
# to the vehicle body frame.  This message will only be available in future 
# INS versions of Swift Products and is not produced by Piksi Multi or Duro.

# GPS Time of Week
# [ms]
uint32 tow
# rotation about the forward axis of the vehicle
# [microdegrees]
int32 roll
# rotation about the rightward axis of the vehicle
# [microdegrees]
int32 pitch
# rotation about the downward axis of the vehicle
# [microdegrees]
int32 yaw
# Estimated standard deviation of roll
# [degrees]
float32 roll_accuracy
# Estimated standard deviation of pitch
# [degrees]
float32 pitch_accuracy
# Estimated standard deviation of yaw
# [degrees]
float32 yaw_accuracy
# Status flags
# []
uint8 flags