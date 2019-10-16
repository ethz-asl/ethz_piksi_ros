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
# MSG_FILEIO_READ_RESP
#
# File read from the file system (host <= device)
#
# The file read message reads a certain length (up to 255 bytes)
# from a given offset into a file, and returns the data in a
# message where the message length field indicates how many bytes
# were succesfully read. The sequence number in the response is
# preserved from the request.

# Read sequence number
# []
uint32 sequence
# Contents of read file
# []
uint8[] contents