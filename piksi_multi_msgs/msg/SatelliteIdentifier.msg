std_msgs/UInt8 sat  # Constellation-specific satellite identifier. This field for Glonass can either be (100+FCN) where FCN is in [-7,+6] or the Slot ID in [1,28]
std_msgs/UInt8 code # Signal constellation, band and code

uint8 CODE_GPS_L1CA  = 0  # GPS L1CA
uint8 CODE_GPS_L2CM  = 1  # GPS L2CM
uint8 CODE_SBAS_L1CA = 2  # SBAS L1CA
uint8 CODE_GLO_L1CA  = 3  # GLO L1CA
uint8 CODE_GLO_L2CA  = 4  # GLO L2CA
uint8 CODE_GPS_L1P   = 5  # GPS L1P
uint8 CODE_GPS_L2P   = 6  # GPS L2P
uint8 CODE_BDS2_B1   = 12 # BDS2 B1
uint8 CODE_BDS2_B2   = 13 # BDS2 B2
uint8 CODE_GAL_E1B   = 14 # GAL E1B
uint8 CODE_GAL_E7I   = 20 # GAL E7I
