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
# MSG_SETTINGS_REGISTER
#
# Register setting and default value (device => host)
#
# This message registers the presence and default value of a setting
# with a settings daemon.  The host should reply with MSG_SETTINGS_WRITE
# for this setting to set the initial value.

# A NULL-terminated and delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE".
# []
string setting