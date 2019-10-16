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

# Observation
# MSG_GLO_BIASES
#
# GLONASS L1/L2 Code-Phase biases
#
# The GLONASS L1/L2 Code-Phase biases allows to perform
# GPS+GLONASS integer ambiguity resolution for baselines
# with mixed receiver types (e.g. receiver of different
# manufacturers)

# GLONASS FDMA signals mask
# [boolean]
uint8 mask
# GLONASS L1 C/A Code-Phase Bias
# [m * 0.02]
int16 l1ca_bias
# GLONASS L1 P Code-Phase Bias
# [m * 0.02]
int16 l1p_bias
# GLONASS L2 C/A Code-Phase Bias
# [m * 0.02]
int16 l2ca_bias
# GLONASS L2 P Code-Phase Bias
# [m * 0.02]
int16 l2p_bias