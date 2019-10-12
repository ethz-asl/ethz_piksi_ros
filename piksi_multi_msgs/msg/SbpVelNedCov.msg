# MSG VEL NED COV — 0x0212 — 530

std_msgs/UInt32  tow                      # GPS Time of Week
std_msgs/Float64 n                        # Velocity North coordinate
std_msgs/Float64 e                        # Velocity East coordinate
std_msgs/Float64 d                        # Velocity Down coordinate
std_msgs/Float32 cov_n_n                  # Estimated variance of northing
std_msgs/Float32 cov_n_e                  # Estimated covariance of northing and easting
std_msgs/Float32 cov_n_d                  # Estimated covariance of northing and downward measurement
std_msgs/Float32 cov_e_e                  # Estimated variance of easting
std_msgs/Float32 cov_e_d                  # Estimated covariance of easting and downward measurement
std_msgs/Float32 cov_d_d                  # Estimated variance of downward measurement
std_msgs/UInt8   n_sats                   # Number of satellites used in solution
std_msgs/UInt8   velocity_mode            # Velocity mode values
std_msgs/Bool    inertial_navigation_mode # Inertial Navigation Mode values

uint8 VELOCITY_MODE_INVALID          = 0
uint8 VELOCITY_MODE_MEASURED_DOPPLER = 1
uint8 VELOCITY_MODE_COMPUTED_DOPPLER = 2
uint8 VELOCITY_MODE_DEAD_RECKONING   = 3

bool INERTIAL_NAVIGATION_MODE_NONE     = 0
bool INERTIAL_NAVIGATION_MODE_INS_USED = 1
