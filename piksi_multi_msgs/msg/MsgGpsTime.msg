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

# Navigation
# MSG_GPS_TIME
#
# GPS Time
#
# This message reports the GPS time, representing the time since
# the GPS epoch began on midnight January 6, 1980 UTC. GPS time
# counts the weeks and seconds of the week. The weeks begin at the
# Saturday/Sunday transition. GPS week 0 began at the beginning of
# the GPS time scale.
# 
# Within each week number, the GPS time of the week is between
# between 0 and 604800 seconds (=60*60*24*7). Note that GPS time
# does not accumulate leap seconds, and as of now, has a small
# offset from UTC. In a message stream, this message precedes a
# set of other navigation messages referenced to the same time
# (but lacking the ns field) and indicates a more precise time of
# these messages.

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
# Status flags (reserved)
# []
uint8 flags