# MSG OBS — 0x004A — 74

GpsTimeValue     time  # GPS time value
uint8            n_obs # Total number of observations. First nibble is the size of the sequence (n), second nibble is the zero-indexed counter (ith packet of n)
GpsObservation[] obs   # GPS observations
