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
# MSG_EPHEMERIS_BDS
#
# Satellite broadcast ephemeris for BDS
#
# The ephemeris message returns a set of satellite orbit
# parameters that is used to calculate BDS satellite position,
# velocity, and clock offset. Please see the BeiDou Navigation
# Satellite System SIS-ICD Version 2.1, Table 5-9 for more details.

# Values common for all ephemeris types
# []
EphemerisCommonContent common
# Group delay differential for B1
# [s]
float32 tgd1
# Group delay differential for B2
# [s]
float32 tgd2
# Amplitude of the sine harmonic correction term to the orbit radius
# [m]
float32 c_rs
# Amplitude of the cosine harmonic correction term to the orbit radius
# [m]
float32 c_rc
# Amplitude of the cosine harmonic correction term to the argument of latitude
# [rad]
float32 c_uc
# Amplitude of the sine harmonic correction term to the argument of latitude
# [rad]
float32 c_us
# Amplitude of the cosine harmonic correction term to the angle of inclination
# [rad]
float32 c_ic
# Amplitude of the sine harmonic correction term to the angle of inclination
# [rad]
float32 c_is
# Mean motion difference
# [rad/s]
float64 dn
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
# Inclination first derivative
# [rad/s]
float64 inc_dot
# Polynomial clock correction coefficient (clock bias)
# [s]
float64 af0
# Polynomial clock correction coefficient (clock drift)
# [s/s]
float32 af1
# Polynomial clock correction coefficient (rate of clock drift)
# [s/s^2]
float32 af2
# Clock reference
# []
GpsTimeSec toc
# Issue of ephemeris data
# []
uint8 iode
# Issue of clock data
# []
uint16 iodc