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

# Ssr
# MSG_SSR_ORBIT_CLOCK
#
# Precise orbit and clock correction
#
# The precise orbit and clock correction message is
# to be applied as a delta correction to broadcast
# ephemeris and is typically an equivalent to the 1060
# and 1066 RTCM message types

# GNSS reference time of the correction
# []
GpsTimeSec time
# GNSS signal identifier (16 bit)
# []
GnssSignal sid
# Update interval between consecutive corrections. Encoded
# following RTCM DF391 specification.
# []
uint8 update_interval
# IOD of the SSR correction. A change of Issue Of Data
# SSR is used to indicate a change in the SSR
# generating configuration
# []
uint8 iod_ssr
# Issue of broadcast ephemeris data or IODCRC (Beidou)
# []
uint32 iod
# Orbit radial delta correction
# [0.1 mm]
int32 radial
# Orbit along delta correction
# [0.4 mm]
int32 along
# Orbit along delta correction
# [0.4 mm]
int32 cross
# Velocity of orbit radial delta correction
# [0.001 mm/s]
int32 dot_radial
# Velocity of orbit along delta correction
# [0.004 mm/s]
int32 dot_along
# Velocity of orbit cross delta correction
# [0.004 mm/s]
int32 dot_cross
# C0 polynomial coefficient for correction of broadcast satellite clock
# [0.1 mm]
int32 c0
# C1 polynomial coefficient for correction of broadcast satellite clock
# [0.001 mm/s]
int32 c1
# C2 polynomial coefficient for correction of broadcast satellite clock
# [0.00002 mm/s^-2]
int32 c2