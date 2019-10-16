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
# Latency
#
# Receiver-to-base station latency
#
# Statistics on the latency of observations received from the base
# station. As observation packets are received their GPS time is
# compared to the current GPS time calculated locally by the
# receiver to give a precise measurement of the end-to-end
# communication latency in the system.

# Average latency
# [ms]
int32 avg
# Minimum latency
# [ms]
int32 lmin
# Maximum latency
# [ms]
int32 lmax
# Smoothed estimate of the current latency
# [ms]
int32 current