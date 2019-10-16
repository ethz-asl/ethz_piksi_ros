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

# Sbas
# MSG_SBAS_RAW
#
# Raw SBAS data
#
# This message is sent once per second per SBAS satellite. ME checks the
# parity of the data block and sends only blocks that pass the check.

# GNSS signal identifier.
# []
GnssSignal sid
# GPS time-of-week at the start of the data block.
# [ms]
uint32 tow
# SBAS message type (0-63)
# []
uint8 message_type
# Raw SBAS data field of 212 bits (last byte padded with zeros).
# []
uint8[] data