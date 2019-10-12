# MSG MAG RAW — 0x0902 — 2306

std_msgs/UInt32 ms    # Milliseconds since start of GPS week. If the high bit is set, the time is unknown or invalid.
std_msgs/UInt8  tow_f # Milliseconds since start of GPS week, fractional part
std_msgs/Int16  mag_x # Magnetic field in the body frame X axis
std_msgs/Int16  mag_y # Magnetic field in the body frame Y axis
std_msgs/Int16  mag_z # Magnetic field in the body frame Z axis
