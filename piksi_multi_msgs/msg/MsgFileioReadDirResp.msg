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

# File_Io
# MSG_FILEIO_READ_DIR_RESP
#
# Files listed in a directory (host <= device)
#
# The read directory message lists the files in a directory on the
# device's onboard flash file system. Message contains the directory
# listings as a NULL delimited list. The listing is chunked over
# multiple SBP packets and the end of the list is identified by an
# entry containing just the character 0xFF. The sequence number in
# the response is preserved from the request.

# Read sequence number
# []
uint32 sequence
# Contents of read directory
# []
uint8[] contents