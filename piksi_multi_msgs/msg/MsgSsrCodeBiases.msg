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
# MSG_SSR_CODE_BIASES
#
# Precise code biases correction
#
# The precise code biases message is to be added
# to the pseudorange of the corresponding signal
# to get corrected pseudorange. It is typically
# an equivalent to the 1059 and 1065 RTCM message types

# GNSS reference time of the correction
# []
GpsTimeSec time
# GNSS signal identifier (16 bit)
# []
GnssSignal sid
# Update interval between consecutive corrections. Encoded
# following RTCM DF391 specification.
# []
uint8 update_interval
# IOD of the SSR correction. A change of Issue Of Data
# SSR is used to indicate a change in the SSR
# generating configuration
# []
uint8 iod_ssr
# Code biases for the different satellite signals
# []
CodeBiasesContent[] biases