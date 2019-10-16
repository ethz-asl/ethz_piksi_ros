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
# MSG_FILEIO_WRITE_REQ
#
# Write to file (host => device)
#
# The file write message writes a certain length (up to 255 bytes)
# of data to a file at a given offset. Returns a copy of the
# original MSG_FILEIO_WRITE_RESP message to check integrity of
# the write. The sequence number in the request will be returned
# in the response. If message is invalid, a followup MSG_PRINT
# message will print "Invalid fileio write message". A device will
# only  process this message when it is received from sender ID
# 0x42.

# Write sequence number
# []
uint32 sequence
# Offset into the file at which to start writing in bytes
# [bytes]
uint32 offset
# Name of the file to write to
# []
string filename
# Variable-length array of data to write
# []
uint8[] data