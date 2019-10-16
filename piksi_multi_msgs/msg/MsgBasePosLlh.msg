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
# MSG_BASE_POS_LLH
#
# Base station position
#
# The base station position message is the position reported by
# the base station itself. It is used for pseudo-absolute RTK
# positioning, and is required to be a high-accuracy surveyed
# location of the base station. Any error here will result in an
# error in the pseudo-absolute position output.

# Latitude
# [deg]
float64 lat
# Longitude
# [deg]
float64 lon
# Height
# [m]
float64 height