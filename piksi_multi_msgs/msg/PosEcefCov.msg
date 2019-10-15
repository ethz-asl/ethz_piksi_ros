# MSG POS ECEF COV — 0x0214 — 532

uint32                 tow             # GPS Time of Week
geometry_msgs/Point    coordinate_ecef # ECEF coordinate
CovCartesian           cov             # Estimated covariance of position
uint8                  n_sats          # Number of satellites used in solution
FixMode                fix_mode        # Fix mode
InertialNavigationMode ins_mode        # Inertial navigation mode
