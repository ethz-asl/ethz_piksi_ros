# MSG EXT EVENT — 0x0101 — 257

GpsTimeValue time    # GPS time
bool         value  # New level of pin
bool         quality # GPS time quality
uint8        pin     # Pin number. 0..9 = DEBUG0..9.

bool QUALITY_UNKNOWN = 0 # No NAV solution.
bool QUALITY_GOOD    = 1 # 1 Microsecond accuracy.

bool VALUE_LOW  = 0 # Falling edge.
bool VALUE_HIGH = 1 # Rising edge
