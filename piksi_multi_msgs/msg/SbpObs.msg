# MSG OBS — 0x004A — 74

std_msgs/UInt32              tow         # Milliseconds since start of GPS week
std_msgs/Int32               ns_residual # Nanosecond residual of millisecond-rounded TOW (ranges from -500000 to 500000)
std_msgs/UInt16              wn          # GPS week number
std_msgs/UInt8               n_obs       # Total number of observations. First nibble is the size of the sequence (n), second nibble is the zero-indexed counter (ith packet of n)
piksi_multi_msgs/SbpGpsObs[] obs         # GPS observations
