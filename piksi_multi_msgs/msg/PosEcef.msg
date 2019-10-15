# MSG POS ECEF — 0x0209 — 521

uint32                 tow        # GPS Time of Week
PointEcef              coordinate # ECEF coordinate
float64                accuracy   # Estimated standard deviation of position
uint8                  n_sats     # Number of satellites used in solution
FixMode                fix_mode   # Fix mode
InertialNavigationMode ins_mode   # Inertial navigation mode
