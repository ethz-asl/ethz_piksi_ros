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

# Piksi
# MSG_SPECAN
#
# Spectrum analyzer
#
# Spectrum analyzer packet.

# Channel ID
# []
uint16 channel_tag
# Receiver time of this observation
# []
GpsTime t
# Reference frequency of this packet
# [MHz]
float32 freq_ref
# Frequency step of points in this packet
# [MHz]
float32 freq_step
# Reference amplitude of this packet
# [dB]
float32 amplitude_ref
# Amplitude unit value of points in this packet
# [dB]
float32 amplitude_unit
# Amplitude values (in the above units) of points in this packet
# []
uint8[] amplitude_value