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
# GnssCapb
#
# GNSS capabilities masks
#


# GPS SV active mask
# []
uint64 gps_active
# GPS L2C active mask
# []
uint64 gps_l2c
# GPS L5 active mask
# []
uint64 gps_l5
# GLO active mask
# []
uint32 glo_active
# GLO L2OF active mask
# []
uint32 glo_l2of
# GLO L3 active mask
# []
uint32 glo_l3
# SBAS active mask (PRNs 120..158, AN 7/62.2.2-18/18 Table B-23,
# https://www.caat.or.th/wp-content/uploads/2018/03/SL-2018.18.E-1.pdf)
# []
uint64 sbas_active
# SBAS L5 active mask (PRNs 120..158, AN 7/62.2.2-18/18 Table B-23,
# https://www.caat.or.th/wp-content/uploads/2018/03/SL-2018.18.E-1.pdf)
# []
uint64 sbas_l5
# BDS active mask
# []
uint64 bds_active
# BDS D2NAV active mask
# []
uint64 bds_d2nav
# BDS B2 active mask
# []
uint64 bds_b2
# BDS B2A active mask
# []
uint64 bds_b2a
# QZSS active mask
# []
uint32 qzss_active
# GAL active mask
# []
uint64 gal_active
# GAL E5 active mask
# []
uint64 gal_e5