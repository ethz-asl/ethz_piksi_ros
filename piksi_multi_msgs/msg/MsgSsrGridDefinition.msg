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
# MSG_SSR_GRID_DEFINITION
#
# None
#
# Definition of the grid for STEC and tropo messages

# Header of a Gridded Correction message
# []
GridDefinitionHeader header
# Run Length Encode list of quadrants that contain valid data.
# The spec describes the encoding scheme in detail, but
# essentially the index of the quadrants that contain transitions between
# valid and invalid (and vice versa) are encoded as u8 integers.
# []
uint8[] rle_list