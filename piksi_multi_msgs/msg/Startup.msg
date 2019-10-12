# MSG STARTUP — 0xFF00 — 65280

std_msgs/UInt8 cause        # Cause of startup
std_msgs/UInt8 startup_type # Startup type

uint8 CAUSE_POWER_ON       = 0
uint8 CAUSE_SOFTWARE_RESET = 1
uint8 CAUSE_WATCHDOG_RESET = 2

uint8 STARTUP_TYPE_COLD_START = 0
uint8 STARTUP_TYPE_WARM_START = 1
uint8 STARTUP_TYPE_HOT_START  = 2
