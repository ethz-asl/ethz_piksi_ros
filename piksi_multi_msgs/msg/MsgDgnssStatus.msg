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

# System
# MSG_DGNSS_STATUS
#
# Status of received corrections
#
# This message provides information about the receipt of Differential
# corrections.  It is expected to be sent with each receipt of a complete
# corrections packet.

# Status flags
# []
uint8 flags
# Latency of observation receipt
# [deci-seconds]
uint16 latency
# Number of signals from base station
# []
uint8 num_signals
# Corrections source string
# []
string source