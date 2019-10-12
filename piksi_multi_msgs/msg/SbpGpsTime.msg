# MSG GPS TIME — 0x0102 — 258

std_msgs/UInt16 wn          # GPS week number
std_msgs/UInt32 ms          # GPS time of week rounded to the nearest millisecond
std_msgs/Int32  ns_residual # Nanosecond residual of millisecond-rounded TOW (ranges from -500000 to 500000)
std_msgs/UInt8  time_source # Status flags (reserved)

uint8 TIME_SOURCE_NONE          = 0 # invalid
uint8 TIME_SOURCE_GNSS_SOLUTION = 0
uint8 TIME_SOURCE_PROPAGATED    = 0
