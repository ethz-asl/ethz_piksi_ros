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
# TrackingChannelState
#
# Signal tracking channel state
#
# Tracking channel state for a specific satellite signal and
# measured signal power.

# GNSS signal being tracked
# []
GnssSignal sid
# Frequency channel number (GLONASS only)
# []
uint8 fcn
# Carrier-to-Noise density.  Zero implies invalid cn0.
# [dB Hz / 4]
uint8 cn0