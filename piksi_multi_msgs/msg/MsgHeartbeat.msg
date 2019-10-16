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
# MSG_HEARTBEAT
#
# System heartbeat message
#
# The heartbeat message is sent periodically to inform the host
# or other attached devices that the system is running. It is
# used to monitor system malfunctions. It also contains status
# flags that indicate to the host the status of the system and
# whether it is operating correctly. Currently, the expected
# heartbeat interval is 1 sec.
# 
# The system error flag is used to indicate that an error has
# occurred in the system. To determine the source of the error,
# the remaining error flags should be inspected.

# Status flags
# []
uint32 flags