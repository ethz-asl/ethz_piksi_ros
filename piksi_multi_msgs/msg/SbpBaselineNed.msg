# MSG BASELINE NED — 0x020C — 524

std_msgs/UInt32  tow                      # GPS Time of Week
std_msgs/Float64 n                        # Baseline North coordinate
std_msgs/Float64 e                        # Baseline East coordinate
std_msgs/Float64 d                        # Baseline Down coordinate
std_msgs/UInt16  h_accuracy               # Horizontal position estimated standard deviation
std_msgs/UInt16  v_accuracy               # Vertical position estimated standard deviation
std_msgs/UInt8   n_sats                   # Number of satellites used in solution
std_msgs/UInt8   fix_mode                 # Fix mode values
std_msgs/Bool    inertial_navigation_mode # Inertial Navigation Mode values

uint8 FIX_MODE_INVALID   = 0
uint8 FIX_MODE_DGNSS     = 2
uint8 FIX_MODE_FLOAT_RTK = 3
uint8 FIX_MODE_FIXED_RTK = 4

bool INERTIAL_NAVIGATION_MODE_NONE     = 0
bool INERTIAL_NAVIGATION_MODE_INS_USED = 1
