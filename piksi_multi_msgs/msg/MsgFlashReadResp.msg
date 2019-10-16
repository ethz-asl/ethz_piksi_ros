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

# Flash
# MSG_FLASH_READ_RESP
#
# Read STM or M25 flash address response (host <= device).
#
# The flash read message reads a set of addresses of either the
# STM or M25 onboard flash. The device replies with a
# MSG_FLASH_READ_RESP message containing either the read data on
# success or a MSG_FLASH_DONE message containing the return code
# FLASH_INVALID_LEN (2) if the maximum read size is exceeded or
# FLASH_INVALID_ADDR (3) if the address is outside of the allowed
# range.

# Target flags
# []
uint8 target
# Starting address offset to read from
# [bytes]
uint8[] addr_start
# Length of set of addresses to read, counting up from
# starting address
# [bytes]
uint8 addr_len