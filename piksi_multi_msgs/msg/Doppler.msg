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
# Doppler
#
# GNSS doppler measurement.
#
# Doppler measurement in Hz represented as a 24-bit
# fixed point number with Q16.8 layout, i.e. 16-bits of whole
# doppler and 8-bits of fractional doppler. This doppler is defined
# as positive for approaching satellites.

# Doppler whole Hz
# [Hz]
int16 i
# Doppler fractional part
# [Hz / 256]
uint8 f