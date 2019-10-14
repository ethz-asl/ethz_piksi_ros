# MSG VEL NED COV — 0x0212 — 530

std_msgs/UInt32                         tow           # GPS Time of Week
piksi_multi_msgs/VectorNed              velocity_ned  # Velocity in NED frame.
piksi_multi_msgs/CovTangentNed          cov           # Estimated covariance
std_msgs/UInt8                          n_sats        # Number of satellites used in solution
piksi_multi_msgs/VelocityMode           velocity_mode # Velocity mode values
piksi_multi_msgs/InertialNavigationMode ins_mode      # Inertial navigation mode
