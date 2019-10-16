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
# PhaseBiasesContent
#
# SSR phase biases corrections for a particular satellite.
#
# Phase biases are to be added to carrier phase measurements.
# The corrections conform with typical RTCMv3 MT1059 and 1065.

# Signal constellation, band and code
# []
uint8 code
# Indicator for integer property
# []
uint8 integer_indicator
# Indicator for two groups of Wide-Lane(s) integer property
# []
uint8 widelane_integer_indicator
# Signal phase discontinuity counter.
# Increased for every discontinuity in phase.
# []
uint8 discontinuity_counter
# Phase bias for specified signal
# [0.1 mm]
int32 bias