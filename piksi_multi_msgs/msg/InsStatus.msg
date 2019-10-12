# MSG INS STATUS — 0xFF03 — 65283

std_msgs/UInt8 mode
std_msgs/Bool  gnss_fix
std_msgs/UInt8 ins_error_value

uint8 MODE_AWAITING_INITIALIZATION  = 0
uint8 MODE_DYNAMICALLY_ALIGNING     = 1
uint8 MODE_READY                    = 2
uint8 MODE_GNSS_OUTAGE_MAX_DURATION = 3

uint8 INS_ERROR_IMU_DATA             = 1
uint8 INS_ERROR_LICENSE              = 2
uint8 INS_ERROR_IMU_CALIBRATION_DATA = 3
