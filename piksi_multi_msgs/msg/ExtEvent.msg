# MSG EXT EVENT — 0x0101 — 257

GpsTimeValue time    # GPS time
bool         quality # GPS time quality
Gpio         pin     # New pin level and number

bool QUALITY_UNKNOWN = 0 # No NAV solution.
bool QUALITY_GOOD    = 1 # 1 Microsecond accuracy.
