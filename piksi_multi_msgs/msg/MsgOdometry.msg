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

# Vehicle
# MSG_ODOMETRY
#
# Vehicle forward (x-axis) velocity
#
# Message representing the x component of vehicle velocity in the user frame at the odometry
# reference point(s) specified by the user. The offset for the odometry reference point and 
# the definition and origin of the user frame are defined through the device settings interface.
# There are 4 possible user-defined sources of this message  which are labeled arbitrarily 
# source 0 through 3.

# Time field representing either milliseconds in the GPS Week or local CPU
# time from the producing system in milliseconds.  See the tow_source flag
# for the exact source of this timestamp.
# [ms]
uint32 tow
# The signed forward component of vehicle velocity.
# [mm/s]
int32 velocity
# Status flags
# []
uint8 flags