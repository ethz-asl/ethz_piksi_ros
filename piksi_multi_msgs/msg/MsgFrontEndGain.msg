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
# MSG_FRONT_END_GAIN
#
# RF AGC status
#
# This message describes the gain of each channel in the receiver frontend. Each 
# gain is encoded as a non-dimensional percentage relative to the maximum range  
# possible for the gain stage of the frontend. By convention, each gain array 
# has 8 entries and the index of the array corresponding to the index of the rf channel 
# in the frontend. A gain of 127 percent encodes that rf channel is not present in the hardware.
# A negative value implies an error for the particular gain stage as reported by the frontend.

# RF gain for each frontend channel
# [percent]
int8[] rf_gain
# Intermediate frequency gain for each frontend channel
# [percent]
int8[] if_gain