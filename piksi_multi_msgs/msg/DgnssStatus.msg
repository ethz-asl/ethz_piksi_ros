# MSG DGNSS STATUS — 0xFF02 — 65282

uint8  differential_type # Differential type values
uint16 latency           # Latency of observation receipt
uint8  num_signals       # Number of signals from base station
string source            # Corrections source string

uint8 DIFFERENTIAL_TYPE_INVALID         = 0
uint8 DIFFERENTIAL_TYPE_CODE_DIFFERENCE = 1
uint8 DIFFERENTIAL_TYPE_RTK             = 2
