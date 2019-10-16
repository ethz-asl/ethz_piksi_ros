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

# System
# MSG_CSAC_TELEMETRY
#
# Experimental telemetry message
#
# The CSAC telemetry message has an implementation defined telemetry string
# from a device. It is not produced or available on general Swift Products.
# It is intended to be a low rate message for status purposes.

# Index representing the type of telemetry in use.  It is implemention defined.
# []
uint8 id
# Comma separated list of values as defined by the index
# []
string telemetry