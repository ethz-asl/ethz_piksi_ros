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
# Period
#
# base station observation message receipt period
#
# Statistics on the period of observations received from the base
# station. As complete observation sets are received, their time
# of reception is compared with the prior set''s time of reception.
# This measurement provides a proxy for link quality as incomplete
# or missing sets will increase the period.  Long periods
# can cause momentary RTK solution outages.

# Average period
# [ms]
int32 avg
# Minimum period
# [ms]
int32 pmin
# Maximum period
# [ms]
int32 pmax
# Smoothed estimate of the current period
# [ms]
int32 current