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
# MSG_BOOTLOADER_HANDSHAKE_RESP
#
# Bootloading handshake response (host <= device)
#
# The handshake message response from the device establishes a
# handshake between the device bootloader and the host. The
# request from the host is MSG_BOOTLOADER_HANDSHAKE_REQ.  The
# payload contains the bootloader version number and the SBP
# protocol version number.

# Bootloader flags
# []
uint32 flags
# Bootloader version number
# []
string version