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
# MSG_FLASH_PROGRAM
#
# Program flash addresses
#
# The flash program message programs a set of addresses of either
# the STM or M25 flash. The device replies with either a
# MSG_FLASH_DONE message containing the return code FLASH_OK (0)
# on success, or FLASH_INVALID_LEN (2) if the maximum write size
# is exceeded. Note that the sector-containing addresses must be
# erased before addresses can be programmed.

# Target flags
# []
uint8 target
# Starting address offset to program
# [bytes]
uint8[] addr_start
# Length of set of addresses to program, counting up from
# starting address
# [bytes]
uint8 addr_len
# Data to program addresses with, with length N=addr_len
# []
uint8[] data