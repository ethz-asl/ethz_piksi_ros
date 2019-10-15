# MSG VEL BODY — 0x0213 — 531

uint32                 tow           # GPS Time of Week
Vector3Int32           velocity_body # Velocity body frame
CovCartesian           cov           # Estimated covariance
uint8                  n_sats        # Number of satellites used in solution
VelocityMode           velocity_mode # Velocity mode values
InertialNavigationMode ins_mode      # Inertial navigation mode
