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
# MSG_FLASH_ERASE
#
# Erase sector of device flash memory (host => device).
#
# The flash erase message from the host erases a sector of either
# the STM or M25 onboard flash memory. The device will reply with a
# MSG_FLASH_DONE message containing the return code - FLASH_OK (0)
# on success or FLASH_INVALID_FLASH (1) if the flash specified is
# invalid.

# Target flags
# []
uint8 target
# Flash sector number to erase (0-11 for the STM, 0-15 for
# the M25)
# []
uint32 sector_num