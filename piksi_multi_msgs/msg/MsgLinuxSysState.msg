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
# MSG_LINUX_SYS_STATE
#
# CPU, Memory and Process Starts/Stops
#
# This presents a summary of CPU and memory utilization.

# total system memory
# []
uint16 mem_total
# percent of total cpu currently utilized
# []
uint8 pcpu
# percent of total memory currently utilized
# []
uint8 pmem
# number of processes that started during collection phase
# []
uint16 procs_starting
# number of processes that stopped during collection phase
# []
uint16 procs_stopping
# the count of processes on the system
# []
uint16 pid_count