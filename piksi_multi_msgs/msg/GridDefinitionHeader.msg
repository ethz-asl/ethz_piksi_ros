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
# GridDefinitionHeader
#
# Defines the grid for MSG_SSR_GRIDDED_CORRECTION messages.
#
# Defines the grid for MSG_SSR_GRIDDED_CORRECTION messages.
# Also includes an RLE encoded validity list.

# inverse of region size
# [coded value - see spec]
uint8 region_size_inverse
# area width; see spec for details
# [coded value - 0 means not present (Swift convention)]
uint16 area_width
# encoded latitude of the northwest corner of the grid
# []
uint16 lat_nw_corner_enc
# encoded longitude of the northwest corner of the grid
# []
uint16 lon_nw_corner_enc
# Number of messages in the dataset
# []
uint8 num_msgs
# Postion of this message in the dataset
# []
uint8 seq_num