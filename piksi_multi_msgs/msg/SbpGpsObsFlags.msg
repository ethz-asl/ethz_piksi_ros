std_msgs/Bool pseudorange_valid             # Pseudorange valid values
std_msgs/Bool carrier_phase_valid           # Carrier phase valid values
std_msgs/Bool half_cycle_ambiguity_resolved # Half-cycle ambiguity values
std_msgs/Bool doppler_valid                 # Doppler valid values
std_msgs/Bool raim_exclusion                # RAIM exclusion values

bool PSEUDORANGE_INVALID = False # Invalid pseudorange measurement
bool PSEUDORANGE_VALID = True    # Valid pseudorange measurement and coarse TOW decoded

bool CARRIER_PHASE_INVALID = False # Invalid carrier phase measurement
bool CARRIER_PHASE_VALID = True    # Valid carrier phase measurement

bool HALF_CYCLE_AMBIGUIITY_UNRESOLVED = False # Half cycle phase ambiguity unresolved
bool HALF_CYCLE_AMBIGUIITY_RESOLVED = True    # Half cycle phase ambiguity resolved

bool DOPPLER_INVALID = False # Invalid doppler measurement
bool DOPPLER_VALID = True    # Valid doppler measurement

bool RAIM_NO_EXCLUSION = False # No exclusion
bool RAIM_EXCLUSION    = True  # Measurement was excluded by SPP RAIM, use with care
