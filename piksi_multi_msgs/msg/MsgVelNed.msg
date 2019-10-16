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
# MSG_VEL_NED
#
# Velocity in NED
#
# This message reports the velocity in local North East Down (NED)
# coordinates. The NED coordinate system is defined as the local WGS84
# tangent plane centered at the current position. The full GPS time is
# given by the preceding MSG_GPS_TIME with the matching time-of-week (tow).

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
# Horizontal velocity estimated standard deviation
# [mm/s]
uint16 h_accuracy
# Vertical velocity estimated standard deviation
# [mm/s]
uint16 v_accuracy
# Number of satellites used in solution
# []
uint8 n_sats
# Status flags
# []
uint8 flags