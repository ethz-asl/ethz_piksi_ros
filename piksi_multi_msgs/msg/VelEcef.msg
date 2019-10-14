# MSG VEL ECEF — 0x020D — 525

std_msgs/UInt32                         tow           # GPS Time of Week
geometry_msgs/Vector3                   velocity_ecef # Velocity ECEF coordinates
std_msgs/UInt16                         accuracy      # Velocity estimated standard deviation
std_msgs/UInt8                          n_sats        # Number of satellites used in solution
piksi_multi_msgs/VelocityMode           velocity_mode # Velocity mode values
piksi_multi_msgs/InertialNavigationMode ins_mode      # Inertial navigation mode
