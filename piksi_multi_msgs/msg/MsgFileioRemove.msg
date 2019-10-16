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
# MSG_FILEIO_REMOVE
#
# Delete a file from the file system (host => device)
#
# The file remove message deletes a file from the file system.
# If the message is invalid, a followup MSG_PRINT message will
# print "Invalid fileio remove message". A device will only
# process this message when it is received from sender ID 0x42.

# Name of the file to delete
# []
string filename