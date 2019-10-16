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
# MSG_ALMANAC_GPS
#
# Satellite broadcast ephemeris for GPS
#
# The almanac message returns a set of satellite orbit parameters. Almanac
# data is not very precise and is considered valid for up to several months.
# Please see the Navstar GPS Space Segment/Navigation user interfaces
# (ICD-GPS-200, Chapter 20.3.3.5.1.2 Almanac Data) for more details.

# Values common for all almanac types
# []
AlmanacCommonContent common
# Mean anomaly at reference time
# [rad]
float64 m0
# Eccentricity of satellite orbit
# []
float64 ecc
# Square root of the semi-major axis of orbit
# [m^(1/2)]
float64 sqrta
# Longitude of ascending node of orbit plane at weekly epoch
# [rad]
float64 omega0
# Rate of right ascension
# [rad/s]
float64 omegadot
# Argument of perigee
# [rad]
float64 w
# Inclination
# [rad]
float64 inc
# Polynomial clock correction coefficient (clock bias)
# [s]
float64 af0
# Polynomial clock correction coefficient (clock drift)
# [s/s]
float64 af1