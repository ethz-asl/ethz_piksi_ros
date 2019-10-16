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

# Bootload
# MSG_NAP_DEVICE_DNA_REQ
#
# Read FPGA device ID over UART request (host => device)
#
# The device message from the host reads a unique device
# identifier from the SwiftNAP, an FPGA. The host requests the ID
# by sending a MSG_NAP_DEVICE_DNA_REQ message. The device
# responds with a MSG_NAP_DEVICE_DNA_RESP message with the
# device ID in the payload. Note that this ID is tied to the FPGA,
# and not related to the Piksi's serial number.
