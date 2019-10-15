PseudoRange         P              # Pseudorange observation
CarrierPhaseCycles  L              # Carrier phase cycles
Doppler             D              # Doppler
CarrierToNoise      cn0            # Carrier-to-Noise density. Zero implies invalid cn0
uint8               lock           # Lock timer. This value gives an indication of the time for which a signal has maintained continuous phase lock. Whenever a signal has lost and regained lock, this value is reset to zero. It is encoded according to DF402 from the RTCM 10403.2 Amendment 2 specification. Valid values range from 0 to 15 and the most significant nibble is reserved for future use.
bool                raim_exclusion # RAIM exclusion values, if true measurement was excluded by SPP RAIM, use with care
SatelliteIdentifier sid            # Satellite identifier.
