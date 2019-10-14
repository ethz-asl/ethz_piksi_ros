# MSG VEL BODY — 0x0213 — 531

std_msgs/UInt32                         tow           # GPS Time of Week
geometry_msgs/Vector3                   velocity_body # Velocity body frame
piksi_multi_msgs/CovCartesian           cov           # Estimated covariance
std_msgs/UInt8                          n_sats        # Number of satellites used in solution
piksi_multi_msgs/VelocityMode           velocity_mode # Velocity mode values
piksi_multi_msgs/InertialNavigationMode ins_mode      # Inertial navigation mode
