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

# Acquisition
# AcqSvProfile
#
# Acq perfomance measurement and debug
#
# Profile for a specific SV for debugging purposes
# The message describes SV profile during acquisition time.
# The message is used to debug and measure the performance.

# SV search job type (deep, fallback, etc)
# []
uint8 job_type
# Acquisition status 1 is Success, 0 is Failure
# []
uint8 status
# CN0 value. Only valid if status is '1'
# [dB-Hz*10]
uint16 cn0
# Acquisition integration time
# [ms]
uint8 int_time
# GNSS signal for which acquisition was attempted
# []
GnssSignal sid
# Acq frequency bin width
# [Hz]
uint16 bin_width
# Timestamp of the job complete event
# [ms]
uint32 timestamp
# Time spent to search for sid.code
# [us]
uint32 time_spent
# Doppler range lowest frequency
# [Hz]
int32 cf_min
# Doppler range highest frequency
# [Hz]
int32 cf_max
# Doppler value of detected peak. Only valid if status is '1'
# [Hz]
int32 cf
# Codephase of detected peak. Only valid if status is '1'
# [chips*10]
uint32 cp