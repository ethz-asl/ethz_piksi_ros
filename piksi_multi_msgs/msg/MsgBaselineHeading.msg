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

# Orientation
# MSG_BASELINE_HEADING
#
# Heading relative to True North
#
# This message reports the baseline heading pointing from the base station
# to the rover relative to True North. The full GPS time is given by the
# preceding MSG_GPS_TIME with the matching time-of-week (tow). It is intended
# that time-matched RTK mode is used when the base station is moving.

# GPS Time of Week
# [ms]
uint32 tow
# Heading
# [mdeg]
uint32 heading
# Number of satellites used in solution
# []
uint8 n_sats
# Status flags
# []
uint8 flags