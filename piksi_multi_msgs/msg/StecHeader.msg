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
# STECHeader
#
# Header for MSG_SSR_STEC_CORRECTION message
#
# A full set of STEC information will likely span multiple SBP
# messages, since SBP message a limited to 255 bytes.  The header
# is used to tie multiple SBP messages into a sequence.

# GNSS reference time of the correction
# []
GpsTimeSec time
# Number of messages in the dataset
# []
uint8 num_msgs
# Position of this message in the dataset
# []
uint8 seq_num
# update interval
# [coded field]
uint8 ssr_update_interval
# Update interval between consecutive corrections. Encoded
# following RTCM DF391 specification.
# []
uint8 update_interval
# IOD of the SSR correction. A change of Issue Of Data
# SSR is used to indicate a change in the SSR
# generating configuration.
# []
uint8 iod_ssr