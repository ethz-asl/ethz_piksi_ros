# MSG IMU RAW — 0x0900 — 2304

std_msgs/UInt32 tow     # Milliseconds since start of GPS week. If the high bit is set, the time is unknown or invalid.
std_msgs/UInt8  tow_f   # Milliseconds since start of GPS week, fractional part
std_msgs/Int16  acc_x   # Acceleration in the IMU frame X axis
std_msgs/Int16  acc_y   # Acceleration in the IMU frame Y axis
std_msgs/Int16  acc_z   # Acceleration in the IMU frame Z axis
std_msgs/Int16  gyr_x   # Angular rate around IMU frame X axis
std_msgs/Int16  gyr_y   # Angular rate around IMU frame Y axis
std_msgs/Int16  gyr_z   # Angular rate around IMU frame Z axis
