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

# Gnss
# CarrierPhase
#
# GNSS carrier phase measurement.
#
# Carrier phase measurement in cycles represented as a 40-bit
# fixed point number with Q32.8 layout, i.e. 32-bits of whole
# cycles and 8-bits of fractional cycles. This phase has the
# same sign as the pseudorange.

# Carrier phase whole cycles
# [cycles]
int32 i
# Carrier phase fractional part
# [cycles / 256]
uint8 f