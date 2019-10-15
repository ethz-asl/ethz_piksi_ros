# MSG VEL ECEF — 0x020D — 525

uint32                 tow           # GPS Time of Week
Vector3Int32           velocity_ecef # Velocity ECEF coordinates
uint16                 accuracy      # Velocity estimated standard deviation
uint8                  n_sats        # Number of satellites used in solution
VelocityMode           velocity_mode # Velocity mode values
InertialNavigationMode ins_mode      # Inertial navigation mode
