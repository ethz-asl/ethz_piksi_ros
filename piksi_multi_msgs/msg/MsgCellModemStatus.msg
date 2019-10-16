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

# Piksi
# MSG_CELL_MODEM_STATUS
#
# Cell modem information update message
#
# If a cell modem is present on a piksi device, this message
# will be send periodically to update the host on the status
# of the modem and its various parameters.

# Received cell signal strength in dBm, zero translates to unknown
# [dBm]
int8 signal_strength
# BER as reported by the modem, zero translates to unknown
# []
float32 signal_error_rate