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
# GPSTime
#
# Nanosecond-accurate receiver clock time
#
# A wire-appropriate receiver clock time, defined as the time
# since the beginning of the week on the Saturday/Sunday
# transition. In most cases, observations are epoch aligned
# so ns field will be 0.

# Milliseconds since start of GPS week
# [ms]
uint32 tow
# Nanosecond residual of millisecond-rounded TOW (ranges
# from -500000 to 500000)
# [ns]
int32 ns_residual
# GPS week number
# [week]
uint16 wn