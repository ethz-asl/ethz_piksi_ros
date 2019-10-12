# MSG IMU AUX — 0x0901 — 2305

std_msgs/UInt8  imu_type             # IMU type
std_msgs/Int16  temp                 # Raw IMU temperature
std_msgs/UInt8  accelerometer_range  # Accelerometer Range values [g]
std_msgs/UInt16 gyroscope_range      # Gyro Range values [deg/s]

uint8 IMU_TYPE_BOSCH_BMI60 = 0 # Bosch BMI160
