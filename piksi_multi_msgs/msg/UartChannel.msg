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
# UARTChannel
#
# State of the UART channel
#
# Throughput, utilization, and error counts on the RX/TX buffers
# of this UART channel. The reported percentage values must
# be normalized.

# UART transmit throughput
# [kB/s]
float32 tx_throughput
# UART receive throughput
# [kB/s]
float32 rx_throughput
# UART CRC error count
# []
uint16 crc_error_count
# UART IO error count
# []
uint16 io_error_count
# UART transmit buffer percentage utilization (ranges from
# 0 to 255)
# []
uint8 tx_buffer_level
# UART receive buffer percentage utilization (ranges from
# 0 to 255)
# []
uint8 rx_buffer_level