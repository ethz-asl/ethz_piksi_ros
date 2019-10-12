# MSG POS ECEF COV — 0x0214 — 532

std_msgs/UInt32                         tow        # GPS Time of Week
piksi_multi_msgs/PointEcef              coordinate # ECEF coordinate
piksi_multi_msgs/CovCartesian           cov        # Estimated covariance of position
std_msgs/UInt8                          n_sats     # Number of satellites used in solution
piksi_multi_msgs/GpsFixMode             fix_mode   # Fix mode
piksi_multi_msgs/InertialNavigationMode ins_mode   # Inertial navigation mode
