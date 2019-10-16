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

# Linux
# MSG_LINUX_MEM_STATE
#
# List CPU state on the system
#
# This message indicates the process state of the top 10 heaviest
# consumers of memory on the system.

# sequence of this status message, values from 0-9
# []
uint8 index
# the PID of the process
# []
uint16 pid
# percent of memory used, expressed as a fraction of 256
# []
uint8 pmem
# fixed length string representing the thread name
# []
string tname
# the command line (as much as it fits in the remaining packet)
# []
string cmdline