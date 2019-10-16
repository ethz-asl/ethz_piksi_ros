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

# Settings
# MSG_SETTINGS_WRITE
#
# Write device configuration settings (host => device)
#
# The setting message writes the device configuration for a particular
# setting via A NULL-terminated and NULL-delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE\0" where the '\0' escape sequence denotes 
# the NULL character and where quotation marks are omitted. A device will
# only process to this message when it is received from sender ID 0x42.
# An example string that could be sent to a device is
# "solution\0soln_freq\010\0".

# A NULL-terminated and NULL-delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE\0"
# []
string setting