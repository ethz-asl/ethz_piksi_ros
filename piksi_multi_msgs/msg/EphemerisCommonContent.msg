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
# EphemerisCommonContent
#
# Common fields for every ephemeris message.
#


# GNSS signal identifier (16 bit)
# []
GnssSignal sid
# Time of Ephemerides
# []
GpsTimeSec toe
# User Range Accuracy
# [m]
float32 ura
# Curve fit interval
# [s]
uint32 fit_interval
# Status of ephemeris, 1 = valid, 0 = invalid
# []
uint8 valid
# Satellite health status.
# GPS: ICD-GPS-200, chapter 20.3.3.3.1.4
# SBAS: 0 = valid, non-zero = invalid
# GLO: 0 = valid, non-zero = invalid
# []
uint8 health_bits