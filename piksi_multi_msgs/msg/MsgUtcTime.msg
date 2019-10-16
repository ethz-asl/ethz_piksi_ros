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
# MSG_UTC_TIME
#
# UTC Time
#
# This message reports the Universal Coordinated Time (UTC).  Note the flags
# which indicate the source of the UTC offset value and source of the time fix.

# Indicates source and time validity
# []
uint8 flags
# GPS time of week rounded to the nearest millisecond
# [ms]
uint32 tow
# Year
# [year]
uint16 year
# Month (range 1 .. 12)
# [months]
uint8 month
# days in the month (range 1-31)
# [day]
uint8 day
# hours of day (range 0-23)
# [hours]
uint8 hours
# minutes of hour (range 0-59)
# [minutes]
uint8 minutes
# seconds of minute (range 0-60) rounded down
# [seconds]
uint8 seconds
# nanoseconds of second (range 0-999999999)
# [nanoseconds]
uint32 ns