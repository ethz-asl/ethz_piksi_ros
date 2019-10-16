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

# Ext_Events
# MSG_EXT_EVENT
#
# Reports timestamped external pin event
#
# Reports detection of an external event, the GPS time it occurred,
# which pin it was and whether it was rising or falling.

# GPS week number
# [weeks]
uint16 wn
# GPS time of week rounded to the nearest millisecond
# [ms]
uint32 tow
# Nanosecond residual of millisecond-rounded TOW (ranges
# from -500000 to 500000)
# [ns]
int32 ns_residual
# Flags
# []
uint8 flags
# Pin number.  0..9 = DEBUG0..9.
# []
uint8 pin