# MSG BASELINE ECEF — 0x020B — 523

std_msgs/UInt32  tow                      # GPS Time of Week
std_msgs/Float64 x                        # Baseline ECEF X coordinate
std_msgs/Float64 y                        # Baseline ECEF Y coordinate
std_msgs/Float64 z                        # Baseline ECEF Z coordinate
std_msgs/UInt16  accuracy                 # Position estimated standard deviation
std_msgs/UInt8   n_sats                   # Number of satellites used in solution
std_msgs/UInt8   fix_mode                 # Fix mode values
std_msgs/Bool    inertial_navigation_mode # Inertial Navigation Mode values

uint8 FIX_MODE_INVALID   = 0
uint8 FIX_MODE_DGNSS     = 2
uint8 FIX_MODE_FLOAT_RTK = 3
uint8 FIX_MODE_FIXED_RTK = 4

bool INERTIAL_NAVIGATION_MODE_NONE     = 0
bool INERTIAL_NAVIGATION_MODE_INS_USED = 1
