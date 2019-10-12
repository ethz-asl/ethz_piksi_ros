# MSG DGNSS STATUS — 0xFF02 — 65282

std_msgs/UInt8  differential_type # Differential type values
std_msgs/UInt16 latency           # Latency of observation receipt
std_msgs/UInt8  num_signals       # Number of signals from base station
std_msgs/String source            # Corrections source string

uint8 DIFFERENTIAL_TYPE_INVALID         = 0
uint8 DIFFERENTIAL_TYPE_CODE_DIFFERENCE = 1
uint8 DIFFERENTIAL_TYPE_RTK             = 2
