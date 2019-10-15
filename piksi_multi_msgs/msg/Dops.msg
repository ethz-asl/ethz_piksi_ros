# MSG DOPS — 0x0208 — 520

uint32  tow            # GPS Time of Week
uint16  gdop           # Geometric Dilution of Precision
uint16  pdop           # Position Dilution of Precision
uint16  tdop           # Time Dilution of Precision
uint16  hdop           # Horizontal Dilution of Precision
uint16  vdop           # Vertical Dilution of Precision
FixMode fix_mode       # Indicates the position solution with which the DOPS message corresponds
bool    raim_exclusion # RAIM exclusion values, if true measurement was excluded by SPP RAIM, use with care
