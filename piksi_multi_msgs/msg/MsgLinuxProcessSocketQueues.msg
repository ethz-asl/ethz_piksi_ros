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
# MSG_LINUX_PROCESS_SOCKET_QUEUES
#
# A list of processes with deep socket queues
#
# Top 10 list of sockets with deep queues.

# sequence of this status message, values from 0-9
# []
uint8 index
# the PID of the process in question
# []
uint16 pid
# the total amount of receive data queued for this process
# []
uint16 recv_queued
# the total amount of send data queued for this process
# []
uint16 send_queued
# A bitfield indicating the socket types used:
#   0x1 (tcp), 0x2 (udp), 0x4 (unix stream), 0x8 (unix dgram), 0x10 (netlink),
#   and 0x8000 (unknown)
# []
uint16 socket_types
# A bitfield indicating the socket states:
#   0x1 (established), 0x2 (syn-sent), 0x4 (syn-recv), 0x8 (fin-wait-1),
#   0x10 (fin-wait-2), 0x20 (time-wait), 0x40 (closed), 0x80 (close-wait),
#   0x100 (last-ack), 0x200 (listen), 0x400 (closing), 0x800 (unconnected),
#   and 0x8000 (unknown)
# []
uint16 socket_states
# Address of the largest queue, remote or local depending on the directionality
# of the connection.
# []
string address_of_largest
# the command line of the process in question
# []
string cmdline