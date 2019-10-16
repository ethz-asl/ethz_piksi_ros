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
# ObservationHeader
#
# Header for observation message.
#
# Header of a GNSS observation message.

# GNSS time of this observation
# []
GpsTime t
# Total number of observations. First nibble is the size
# of the sequence (n), second nibble is the zero-indexed
# counter (ith packet of n)
# []
uint8 n_obs