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

# Observation
# PackedOsrContent
#
# Network correction for a particular satellite signal.
#
# Pseudorange and carrier phase network corrections for a satellite signal.

# Pseudorange observation
# [2 cm]
uint32 P
# Carrier phase observation with typical sign convention.
# [cycles]
CarrierPhase L
# Lock timer. This value gives an indication of the time
# for which a signal has maintained continuous phase lock.
# Whenever a signal has lost and regained lock, this
# value is reset to zero. It is encoded according to DF402 from
# the RTCM 10403.2 Amendment 2 specification.  Valid values range
# from 0 to 15 and the most significant nibble is reserved for future use.
# []
uint8 lock
# Correction flags.
# []
uint8 flags
# GNSS signal identifier (16 bit)
# []
GnssSignal sid
# Slant ionospheric correction standard deviation
# [5 mm]
uint16 iono_std
# Slant tropospheric correction standard deviation
# [5 mm]
uint16 tropo_std
# Orbit/clock/bias correction projected on range standard deviation
# [5 mm]
uint16 range_std