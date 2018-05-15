# Raw data from the magnetometer.

Header header

uint32 tow    # Milliseconds since start of GPS week. If the high bit is set, the time is unknown or invalid.
uint8 tow_f   # Milliseconds since start of GPS week, fractional part.
int16 mag_x   # Magnetic field in the body frame X axis.
int16 mag_y   # Magnetic field in the body frame Y axis.
int16 mag_z   # Magnetic field in the body frame Z axis.
