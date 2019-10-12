# MSG POS LLH COV — 0x0211 — 529

std_msgs/UInt32  tow                      # GPS Time of Week
std_msgs/Float64 lat                      # Latitude
std_msgs/Float64 lon                      # Longitude
std_msgs/Float64 height                   # Height above WGS84 ellipsoid
std_msgs/Float32 cov_n_n                  # Estimated variance of northing
std_msgs/Float32 cov_n_e                  # Estimated covariance of northing and easting
std_msgs/Float32 cov_n_d                  # Estimated covariance of northing and downward measurement
std_msgs/Float32 cov_e_e                  # Estimated variance of easting
std_msgs/Float32 cov_e_d                  # Estimated covariance of easting and downward measurement
std_msgs/Float32 cov_d_d                  # Estimated variance of downward measurement
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
