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
# MSG_POS_LLH
#
# Geodetic Position
#
# This position solution message reports the absolute geodetic
# coordinates and the status (single point vs pseudo-absolute RTK)
# of the position solution. If the rover receiver knows the
# surveyed position of the base station and has an RTK solution,
# this reports a pseudo-absolute position solution using the base
# station position and the rover's RTK baseline vector. The full
# GPS time is given by the preceding MSG_GPS_TIME with the
# matching time-of-week (tow).

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
# Horizontal position estimated standard deviation
# [mm]
uint16 h_accuracy
# Vertical position estimated standard deviation
# [mm]
uint16 v_accuracy
# Number of satellites used in solution.
# []
uint8 n_sats
# Status flags
# []
uint8 flags