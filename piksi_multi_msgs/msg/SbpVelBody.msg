# MSG VEL BODY — 0x0213 — 531

std_msgs/UInt32  tow                      # GPS Time of Week
std_msgs/Float64 x                        # Velocity in x direction
std_msgs/Float64 y                        # Velocity in y direction
std_msgs/Float64 z                        # Velocity in z direction
std_msgs/Float32 cov_x_x                  # Estimated variance of x
std_msgs/Float32 cov_x_y                  # Estimated covariance of x and y
std_msgs/Float32 cov_x_z                  # Estimated covariance of x and z
std_msgs/Float32 cov_y_y                  # Estimated variance of y
std_msgs/Float32 cov_y_z                  # Estimated covariance of y and z
std_msgs/Float32 cov_z_z                  # Estimated variance of z
std_msgs/UInt8   n_sats                   # Number of satellites used in solution
std_msgs/UInt8   velocity_mode            # Velocity mode values
std_msgs/Bool    inertial_navigation_mode # Inertial Navigation Mode values

uint8 VELOCITY_MODE_INVALID          = 0
uint8 VELOCITY_MODE_MEASURED_DOPPLER = 1
uint8 VELOCITY_MODE_COMPUTED_DOPPLER = 2
uint8 VELOCITY_MODE_DEAD_RECKONING   = 3

bool INERTIAL_NAVIGATION_MODE_NONE     = 0
bool INERTIAL_NAVIGATION_MODE_INS_USED = 1
