# MSG OBS — 0x004A — 74

piksi_multi_msgs/GpsTimeValue     time  # GPS time value
std_msgs/UInt8                    n_obs # Total number of observations. First nibble is the size of the sequence (n), second nibble is the zero-indexed counter (ith packet of n)
piksi_multi_msgs/GpsObservation[] obs   # GPS observations
