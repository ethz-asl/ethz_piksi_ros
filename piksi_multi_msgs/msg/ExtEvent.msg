# MSG EXT EVENT — 0x0101 — 257

std_msgs/UInt16 wn          # GPS week number
std_msgs/UInt32 tow         # GPS time of week rounded to the nearest millisecond
std_msgs/Int32  ns_residual # Nanosecond residual of millisecond-rounded TOW (ranges from -500000 to 500000)
std_msgs/Bool   pin_value
std_msgs/Bool   quality
std_msgs/UInt8  pin         # Pin number. 0..9 = DEBUG0..9.

bool PIN_VALUE_LOW   = 0 # Falling edge.
bool PIN_VALUE_HIGH  = 1 # Rising edge
bool QUALITY_UNKNOWN = 0 # No NAV solution.
bool QUALITY_GOOD    = 1 # 1 Microsecond accuracy.
