piksi_multi_msgs/PseudoRange         P              # Pseudorange observation
piksi_multi_msgs/CarrierPhaseCycles  L              # Carrier phase cycles
piksi_multi_msgs/Doppler             D              # Doppler
std_msgs/UInt8                       cn0            # Carrier-to-Noise density. Zero implies invalid cn0
std_msgs/UInt8                       lock           # Lock timer. This value gives an indication of the time for which a signal has maintained continuous phase lock. Whenever a signal has lost and regained lock, this value is reset to zero. It is encoded according to DF402 from the RTCM 10403.2 Amendment 2 specification. Valid values range from 0 to 15 and the most significant nibble is reserved for future use.
std_msgs/Bool                        raim_exclusion # RAIM exclusion values, if true measurement was excluded by SPP RAIM, use with care
piksi_multi_msgs/SatelliteIdentifier sid            # Satellite identifier.

uint8 CN0_INVALID = 0 # invalid
