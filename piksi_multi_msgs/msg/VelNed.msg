# MSG VEL NED — 0x020E — 526

std_msgs/UInt32                         tow           # GPS Time of Week
piksi_multi_msgs/VectorNed              velocity      # Velocity in NED frame.
piksi_multi_msgs/AccuracyTangentPlane   accuracy      # Horizontal and vertical standard deviation
std_msgs/UInt8                          n_sats        # Number of satellites used in solution
piksi_multi_msgs/VelocityMode           velocity_mode # Velocity mode values
piksi_multi_msgs/InertialNavigationMode ins_mode      # Inertial navigation mode
