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
# MSG_BASELINE_NED
#
# Baseline in NED
#
# This message reports the baseline solution in North East Down
# (NED) coordinates. This baseline is the relative vector distance
# from the base station to the rover receiver, and NED coordinate
# system is defined at the local WGS84 tangent plane centered at the
# base station position.  The full GPS time is given by the
# preceding MSG_GPS_TIME with the matching time-of-week (tow).

# GPS Time of Week
# [ms]
uint32 tow
# Baseline North coordinate
# [mm]
int32 n
# Baseline East coordinate
# [mm]
int32 e
# Baseline Down coordinate
# [mm]
int32 d
# Horizontal position estimated standard deviation
# [mm]
uint16 h_accuracy
# Vertical position estimated standard deviation
# [mm]
uint16 v_accuracy
# Number of satellites used in solution
# []
uint8 n_sats
# Status flags
# []
uint8 flags