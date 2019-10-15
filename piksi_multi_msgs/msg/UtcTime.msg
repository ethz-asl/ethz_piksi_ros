# MSG UTC TIME — 0x0103 — 259

GpsTimeSource time_source       # Time source values
uint8         offset_source     # UTC offset source values
uint32        tow               # GPS time of week rounded to the nearest millisecond
uint16        year              # Year
uint8         month             # Month (range 1 .. 12)
uint8         day               # days in the month (range 1-31)
uint8         hours             # hours of day (range 0-23)
uint8         minutes           # minutes of hour (range 0-59)
uint8         seconds           # seconds of minute (range 0-60) rounded down
uint32        ns                # nanoseconds of second (range 0-999999999)

uint8 UTC_OFFSET_SRC_FACTORY_DEFAULT      = 0
uint8 UTC_OFFSET_SRC_NON_VOLATILE_MEMORY  = 1
uint8 UTC_OFFSET_SRC_DECODED_THIS_SESSION = 2
