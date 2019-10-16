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
# MSG_DOPS
#
# Dilution of Precision
#
# This dilution of precision (DOP) message describes the effect of
# navigation satellite geometry on positional measurement
# precision.  The flags field indicated whether the DOP reported
# corresponds to differential or SPP solution.

# GPS Time of Week
# [ms]
uint32 tow
# Geometric Dilution of Precision
# [0.01]
uint16 gdop
# Position Dilution of Precision
# [0.01]
uint16 pdop
# Time Dilution of Precision
# [0.01]
uint16 tdop
# Horizontal Dilution of Precision
# [0.01]
uint16 hdop
# Vertical Dilution of Precision
# [0.01]
uint16 vdop
# Indicates the position solution with which the DOPS message corresponds
# []
uint8 flags