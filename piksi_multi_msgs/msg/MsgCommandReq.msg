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
# MSG_COMMAND_REQ
#
# Execute a command (host => device)
#
# Request the recipient to execute an command.
# Output will be sent in MSG_LOG messages, and the exit
# code will be returned with MSG_COMMAND_RESP.

# Sequence number
# []
uint32 sequence
# Command line to execute
# []
string command