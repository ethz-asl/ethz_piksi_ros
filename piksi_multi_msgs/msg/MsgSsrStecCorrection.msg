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
# MSG_SSR_STEC_CORRECTION
#
# Slant Total Electron Content
#
# The STEC per space vehicle, given as polynomial approximation for
# a given grid.  This should be combined with MSG_SSR_GRIDDED_CORRECTION
# message to get the state space representation of the atmospheric
# delay. It is typically equivalent to the QZSS CLAS Sub Type 8 messages

# Header of a STEC message
# []
StecHeader header
# Array of STEC information for each space vehicle
# []
StecSatElement[] stec_sat_list