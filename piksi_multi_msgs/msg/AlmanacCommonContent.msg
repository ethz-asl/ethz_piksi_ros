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
# AlmanacCommonContent
#
# Common fields for every almanac message.
#


# GNSS signal identifier
# []
GnssSignal sid
# Reference time of almanac
# []
GpsTimeSec toa
# User Range Accuracy
# [m]
float64 ura
# Curve fit interval
# [s]
uint32 fit_interval
# Status of almanac, 1 = valid, 0 = invalid
# []
uint8 valid
# Satellite health status for GPS:
#   - bits 5-7: NAV data health status. See IS-GPS-200H
#     Table 20-VII: NAV Data Health Indications.
#   - bits 0-4: Signal health status. See IS-GPS-200H
#     Table 20-VIII. Codes for Health of SV Signal
#     Components.
# Satellite health status for GLO:
#   See GLO ICD 5.1 table 5.1 for details
#   - bit 0: C(n), "unhealthy" flag that is transmitted within
#     non-immediate data and indicates overall constellation status
#     at the moment of almanac uploading.
#     '0' indicates malfunction of n-satellite.
#     '1' indicates that n-satellite is operational.
#   - bit 1: Bn(ln), '0' indicates the satellite is operational
#     and suitable for navigation.
# []
uint8 health_bits