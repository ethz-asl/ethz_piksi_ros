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
# GridElement
#
# Correction data for a single grid point.
#
# Contains one tropo delay, plus STEC residuals for each satellite at the
# grid point.

# Index of the grid point
# []
uint16 index
# Wet and hydrostatic vertical delays
# []
TroposphericDelayCorrection tropo_delay_correction
# STEC residuals for each satellite
# []
StecResidual[] stec_residuals