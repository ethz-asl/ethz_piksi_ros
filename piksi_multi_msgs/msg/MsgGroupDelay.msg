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
# MSG_GROUP_DELAY
#
# Group Delay
#
# Please see ICD-GPS-200 (30.3.3.3.1.1) for more details.

# Data Predict Time of Week
# []
GpsTimeSec t_op
# GNSS signal identifier
# []
GnssSignal sid
# bit-field indicating validity of the values,
# LSB indicating tgd validity etc.
# 1 = value is valid, 0 = value is not valid.
# []
uint8 valid

# [s * 2^-35]
int16 tgd

# [s * 2^-35]
int16 isc_l1ca

# [s * 2^-35]
int16 isc_l2c