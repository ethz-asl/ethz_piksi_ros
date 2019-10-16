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
# MSG_TRACKING_IQ
#
# Tracking channel correlations
#
# When enabled, a tracking channel can output the correlations at each
# update interval.

# Tracking channel of origin
# []
uint8 channel
# GNSS signal identifier
# []
GnssSignal sid
# Early, Prompt and Late correlations
# []
TrackingChannelCorrelation[] corrs