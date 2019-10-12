# MSG POS ECEF COV — 0x0214 — 532

std_msgs/UInt32  tow                      # GPS Time of Week
std_msgs/Float64 x                        # ECEF X coordinate
std_msgs/Float64 y                        # ECEF Y coordinate
std_msgs/Float64 z                        # ECEF Z coordinate
std_msgs/Float32 cov_x_x                  # Estimated variance of x
std_msgs/Float32 cov_x_y                  # Estimated covariance of x and y
std_msgs/Float32 cov_x_z                  # Estimated covariance of x and z
std_msgs/Float32 cov_y_y                  # Estimated variance of y
std_msgs/Float32 cov_y_z                  # Estimated covariance of y and z
std_msgs/Float32 cov_z_z                  # Estimated variance of z
std_msgs/UInt8   n_sats                   # Number of satellites used in solution
std_msgs/UInt8   fix_mode                 # Fix mode values
std_msgs/Bool    inertial_navigation_mode # Inertial Navigation Mode values

uint8 FIX_MODE_INVALID   = 0
uint8 FIX_MODE_SPP       = 1
uint8 FIX_MODE_DGNSS     = 2
uint8 FIX_MODE_FLOAT_RTK = 3
uint8 FIX_MODE_FIXED_RTK = 4
uint8 FIX_MODE_UNDEFINED = 5
uint8 FIX_MODE_SBAS      = 6

bool INERTIAL_NAVIGATION_MODE_NONE     = 0
bool INERTIAL_NAVIGATION_MODE_INS_USED = 1
