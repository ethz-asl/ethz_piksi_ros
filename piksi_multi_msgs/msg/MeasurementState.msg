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

# Tracking
# MeasurementState
#
# Measurement Engine signal tracking channel state
#
# Measurement Engine tracking channel state for a specific satellite signal
# and measured signal power.
# The mesid field for Glonass can either
# carry the FCN as 100 + FCN where FCN is in [-7, +6] or
# the Slot ID (from 1 to 28)

# Measurement Engine GNSS signal being tracked (carries either Glonass FCN or SLOT)
# []
GnssSignal mesid
# Carrier-to-Noise density.  Zero implies invalid cn0.
# [dB Hz / 4]
uint8 cn0