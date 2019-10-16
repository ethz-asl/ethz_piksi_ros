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

# Ssr
# MSG_SSR_GRIDDED_CORRECTION
#
# Gridded troposphere and STEC residuals
#
# STEC residuals are per space vehicle, tropo is not.
# It is typically equivalent to the QZSS CLAS Sub Type 9 messages

# Header of a Gridded Correction message
# []
GriddedCorrectionHeader header
# Tropo and STEC residuals for the given grid point
# []
GridElement element