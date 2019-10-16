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

# Observation
# MSG_EPHEMERIS_GLO
#
# Satellite broadcast ephemeris for GLO
#
# The ephemeris message returns a set of satellite orbit
# parameters that is used to calculate GLO satellite position,
# velocity, and clock offset. Please see the GLO ICD 5.1 "Table 4.5
# Characteristics of words of immediate information (ephemeris parameters)"
# for more details.

# Values common for all ephemeris types
# []
EphemerisCommonContent common
# Relative deviation of predicted carrier frequency from nominal
# []
float32 gamma
# Correction to the SV time
# [s]
float32 tau
# Equipment delay between L1 and L2
# [s]
float32 d_tau
# Position of the SV at tb in PZ-90.02 coordinates system
# [m]
float64[] pos
# Velocity vector of the SV at tb in PZ-90.02 coordinates system
# [m/s]
float64[] vel
# Acceleration vector of the SV at tb in PZ-90.02 coordinates sys
# [m/s^2]
float32[] acc
# Frequency slot. FCN+8 (that is [1..14]). 0 or 0xFF for invalid
# []
uint8 fcn
# Issue of ephemeris data
# []
uint8 iod