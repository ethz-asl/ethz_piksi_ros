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
# MSG_FLASH_DONE
#
# Flash response message (host <= device).
#
# This message defines success or failure codes for a variety of
# flash memory requests from the host to the device. Flash read
# and write messages, such as MSG_FLASH_READ_REQ, or
# MSG_FLASH_PROGRAM, may return this message on failure.

# Response flags
# []
uint8 response