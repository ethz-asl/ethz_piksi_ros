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
# MSG_ALMANAC_GLO
#
# Satellite broadcast ephemeris for GLO
#
# The almanac message returns a set of satellite orbit parameters. Almanac
# data is not very precise and is considered valid for up to several months.
# Please see the GLO ICD 5.1 "Chapter 4.5 Non-immediate information and
# almanac" for details.

# Values common for all almanac types
# []
AlmanacCommonContent common
# Longitude of the first ascending node of the orbit in PZ-90.02
# coordinate system
# [rad]
float64 lambda_na
# Time of the first ascending node passage
# [s]
float64 t_lambda_na
# Value of inclination at instant of t_lambda
# [rad]
float64 i
# Value of Draconian period at instant of t_lambda
# [s/orbital period]
float64 t
# Rate of change of the Draconian period
# [s/(orbital period^2)]
float64 t_dot
# Eccentricity at instant of t_lambda
# []
float64 epsilon
# Argument of perigee at instant of t_lambda
# [rad]
float64 omega