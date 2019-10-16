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

# Ssr
# STECSatElement
#
# None
#
# STEC polynomial for the given satellite.

# Unique space vehicle identifier
# []
SvId sv_id
# Quality of the STEC data. Encoded following RTCM DF389 specifcation
# but as TECU instead of m.
# []
uint8 stec_quality_indicator
# Coefficents of the STEC polynomial in the order of C00, C01, C10, C11
# [C00 = 0.05 TECU, others = 0.02 TECU/deg]
int16[] stec_coeff