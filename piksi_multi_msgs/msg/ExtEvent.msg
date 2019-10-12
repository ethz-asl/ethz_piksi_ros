# MSG EXT EVENT — 0x0101 — 257

piksi_multi_msgs/GpsTimeValue   time    # GPS time
std_msgs/Bool                   quality # GPS time quality
piksi_multi_msgs/Gpio           pin     # New pin level and number

bool QUALITY_UNKNOWN = 0 # No NAV solution.
bool QUALITY_GOOD    = 1 # 1 Microsecond accuracy.
