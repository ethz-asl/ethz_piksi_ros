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
# MSG_EPHEMERIS_SBAS
#
# Satellite broadcast ephemeris for SBAS
#


# Values common for all ephemeris types
# []
EphemerisCommonContent common
# Position of the GEO at time toe
# [m]
float64[] pos
# Velocity of the GEO at time toe
# [m/s]
float32[] vel
# Acceleration of the GEO at time toe
# [m/s^2]
float32[] acc
# Time offset of the GEO clock w.r.t. SBAS Network Time
# [s]
float32 a_gf0
# Drift of the GEO clock w.r.t. SBAS Network Time
# [s/s]
float32 a_gf1