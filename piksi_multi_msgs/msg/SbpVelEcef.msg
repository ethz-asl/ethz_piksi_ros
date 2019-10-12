# MSG VEL ECEF — 0x020D — 525

std_msgs/UInt32  tow                      # GPS Time of Week
std_msgs/Float64 x                        # Velocity ECEF X coordinate
std_msgs/Float64 y                        # Velocity ECEF Y coordinate
std_msgs/Float64 z                        # Velocity ECEF Z coordinate
std_msgs/UInt16  accuracy                 # Velocity estimated standard deviation
std_msgs/UInt8   n_sats                   # Number of satellites used in solution
std_msgs/UInt8   velocity_mode            # Velocity mode values
std_msgs/Bool    inertial_navigation_mode # Inertial Navigation Mode values

uint8 VELOCITY_MODE_INVALID          = 0
uint8 VELOCITY_MODE_MEASURED_DOPPLER = 1
uint8 VELOCITY_MODE_COMPUTED_DOPPLER = 2
uint8 VELOCITY_MODE_DEAD_RECKONING   = 3

bool INERTIAL_NAVIGATION_MODE_NONE     = 0
bool INERTIAL_NAVIGATION_MODE_INS_USED = 1
