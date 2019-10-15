# MSG VEL ECEF COV — 0x0215 — 533

uint32                 tow           # GPS Time of Week
geometry_msgs/Vector3  velocity_ecef # Velocity ECEF coordinates
CovCartesian           cov           # Estimated covariance
uint8                  n_sats        # Number of satellites used in solution
VelocityMode           velocity_mode # Velocity mode values
InertialNavigationMode ins_mode      # Inertial navigation mode
