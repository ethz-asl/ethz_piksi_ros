# MSG POS LLH COV — 0x0211 — 529

std_msgs/UInt32                         tow        # GPS Time of Week
piksi_multi_msgs/PointWgs84             coordinate # WGS84 coordinate
piksi_multi_msgs/CovTangentNed          cov        # Estimated covariance of WGS84 coordinate
std_msgs/UInt8                          n_sats     # Number of satellites used in solution
piksi_multi_msgs/GpsFixMode             fix_mode   # Fix mode
piksi_multi_msgs/InertialNavigationMode ins_mode   # Inertial navigation mode
