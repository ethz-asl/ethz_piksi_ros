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
# MSG_CSAC_TELEMETRY_LABELS
#
# Experimental telemetry message labels
#
# The CSAC telemetry message provides labels for each member of the string
# produced by MSG_CSAC_TELEMETRY. It should be provided by a device at a lower
# rate than the MSG_CSAC_TELEMETRY.

# Index representing the type of telemetry in use.  It is implemention defined.
# []
uint8 id
# Comma separated list of telemetry field values
# []
string telemetry_labels