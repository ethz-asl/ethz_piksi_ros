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

# Acquisition
# MSG_ACQ_RESULT
#
# Satellite acquisition result
#
# This message describes the results from an attempted GPS signal
# acquisition search for a satellite PRN over a code phase/carrier
# frequency range. It contains the parameters of the point in the
# acquisition search space with the best carrier-to-noise (CN/0)
# ratio.

# CN/0 of best point
# [dB Hz]
float32 cn0
# Code phase of best point
# [chips]
float32 cp
# Carrier frequency of best point
# [hz]
float32 cf
# GNSS signal for which acquisition was attempted
# []
GnssSignal sid