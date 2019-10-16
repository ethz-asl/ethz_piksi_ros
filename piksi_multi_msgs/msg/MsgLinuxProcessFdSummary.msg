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
# MSG_LINUX_PROCESS_FD_SUMMARY
#
# Summary of open file descriptors on the system
#
# Summary of open file descriptors on the system.

# count of total FDs open on the system
# []
uint32 sys_fd_count
# A null delimited list of strings which alternates between
# a string representation of the process count and the file
# name whose count it being reported.  That is, in C string
# syntax "32\0/var/log/syslog\012\0/tmp/foo\0" with the end
# []
string most_opened