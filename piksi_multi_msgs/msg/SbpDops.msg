# MSG DOPS — 0x0208 — 520

std_msgs/UInt32 tow         # GPS Time of Week
std_msgs/UInt16 gdop        # Geometric Dilution of Precision
std_msgs/UInt16 pdop        # Position Dilution of Precision
std_msgs/UInt16 tdop        # Time Dilution of Precision
std_msgs/UInt16 hdop        # Horizontal Dilution of Precision
std_msgs/UInt16 vdop        # Vertical Dilution of Precision
std_msgs/UInt8  fix_mode    # Fix mode values
std_msgs/Bool   raim_repair # RAIM repair flag

uint8 FIX_MODE_INVALID   = 0
uint8 FIX_MODE_SPP       = 1
uint8 FIX_MODE_DGNSS     = 2
uint8 FIX_MODE_FLOAT_RTK = 3
uint8 FIX_MODE_FIXED_RTK = 4
uint8 FIX_MODE_UNDEFINED = 5
uint8 FIX_MODE_SBAS      = 6
