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
# GnssSignal
#
# Represents all the relevant information about the signal
#
# Signal identifier containing constellation, band, and satellite identifier

# Constellation-specific satellite identifier. This field for Glonass can  
# either be (100+FCN) where FCN is in [-7,+6] or 
# the Slot ID in [1,28]
# []
uint8 sat
# Signal constellation, band and code
# []
uint8 code