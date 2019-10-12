# MSG UTC TIME — 0x0103 — 259

piksi_multi_msgs/GpsTimeSource   time_source       # Time source values
std_msgs/UInt8                   offset_source     # UTC offset source values
std_msgs/UInt32                  tow               # GPS time of week rounded to the nearest millisecond
std_msgs/UInt16                  year              # Year
std_msgs/UInt8                   month             # Month (range 1 .. 12)
std_msgs/UInt8                   day               # days in the month (range 1-31)
std_msgs/UInt8                   hours             # hours of day (range 0-23)
std_msgs/UInt8                   minutes           # minutes of hour (range 0-59)
std_msgs/UInt8                   seconds           # seconds of minute (range 0-60) rounded down
std_msgs/UInt32                  ns                # nanoseconds of second (range 0-999999999)

uint8 UTC_OFFSET_SRC_FACTORY_DEFAULT      = 0
uint8 UTC_OFFSET_SRC_NON_VOLATILE_MEMORY  = 1
uint8 UTC_OFFSET_SRC_DECODED_THIS_SESSION = 2
