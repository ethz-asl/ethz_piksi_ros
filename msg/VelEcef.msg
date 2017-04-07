# This message reports the velocity in Earth Centered Earth Fixed (ECEF) coordinates. The full GPS time
# is given by the preceding MSG GPS TIME with the matching time-of-week (tow).

Header header

uint32 tow
int32 x
int32 y
int32 z
uint16 accuracy
uint8 n_sats
uint8 flags
