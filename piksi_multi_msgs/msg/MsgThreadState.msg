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
# MSG_THREAD_STATE
#
# State of an RTOS thread
#
# The thread usage message from the device reports real-time
# operating system (RTOS) thread usage statistics for the named
# thread. The reported percentage values must be normalized.

# Thread name (NULL terminated)
# []
string name
# Percentage cpu use for this thread. Values range from 0
# - 1000 and needs to be renormalized to 100
# []
uint16 cpu
# Free stack space for this thread
# [bytes]
uint32 stack_free