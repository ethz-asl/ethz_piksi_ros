std_msgs/UInt32                         P     # Pseudorange observation
piksi_multi_msgs/SbpCarrierPhaseCycles  L     # Carrier phase cycles
piksi_multi_msgs/SbpDoppler             D     # Carrier phase cycles
std_msgs/UInt8                          cn0   # Carrier-to-Noise density. Zero implies invalid cn0
std_msgs/UInt8                          lock  # Lock timer. This value gives an indication of the time for which a signal has maintained continuous phase lock. Whenever a signal has lost and regained lock, this value is reset to zero. It is encoded according to DF402 from the RTCM 10403.2 Amendment 2 specification. Valid values range from 0 to 15 and the most significant nibble is reserved for future use.
piksi_multi_msgs/SbpGpsObsFlags         flags # Measurement status flags. A bit field of flags providing the status of this observation. If no flags are set it means only the Cn0 estimate for the signal is valid.
piksi_multi_msgs/SbpSatelliteIdentifier sid   # Satellite identifier.

uint8 CN0_INVALID = 0 # invalid
