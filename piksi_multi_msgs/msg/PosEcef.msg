# MSG POS ECEF — 0x0209 — 521

std_msgs/UInt32                         tow        # GPS Time of Week
piksi_multi_msgs/PointEcef              coordinate # ECEF coordinate
std_msgs/UInt16                         accuracy   # Estimated standard deviation of position
std_msgs/UInt8                          n_sats     # Number of satellites used in solution
piksi_multi_msgs/GpsFixMode             fix_mode   # Fix mode
piksi_multi_msgs/InertialNavigationMode ins_mode   # Inertial navigation mode
