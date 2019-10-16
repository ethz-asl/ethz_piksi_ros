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
# MSG_IONO
#
# Iono corrections
#
# The ionospheric parameters which allow the "L1 only" or "L2 only" user to
# utilize the ionospheric model for computation of the ionospheric delay.
# Please see ICD-GPS-200 (Chapter 20.3.3.5.1.7) for more details.

# Navigation Message Correction Table Valitidy Time
# []
GpsTimeSec t_nmct

# [s]
float64 a0

# [s/semi-circle]
float64 a1

# [s/(semi-circle)^2]
float64 a2

# [s/(semi-circle)^3]
float64 a3

# [s]
float64 b0

# [s/semi-circle]
float64 b1

# [s/(semi-circle)^2]
float64 b2

# [s/(semi-circle)^3]
float64 b3