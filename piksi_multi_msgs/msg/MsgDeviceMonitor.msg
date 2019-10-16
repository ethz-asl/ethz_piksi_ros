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

# Piksi
# MSG_DEVICE_MONITOR
#
# Device temperature and voltage levels
#
# This message contains temperature and voltage level measurements from the
# processor's monitoring system and the RF frontend die temperature if
# available.

# Device V_in
# [V / 1000]
int16 dev_vin
# Processor V_int
# [V / 1000]
int16 cpu_vint
# Processor V_aux
# [V / 1000]
int16 cpu_vaux
# Processor temperature
# [degrees C / 100]
int16 cpu_temperature
# Frontend temperature (if available)
# [degrees C / 100]
int16 fe_temperature