# MSG IMU AUX — 0x0901 — 2305

uint8  imu_type             # IMU type
int16  temp                 # Raw IMU temperature
uint8  accelerometer_range  # Accelerometer Range values [g]
uint16 gyroscope_range      # Gyro Range values [deg/s]

uint8 IMU_TYPE_BOSCH_BMI60 = 0 # Bosch BMI160
