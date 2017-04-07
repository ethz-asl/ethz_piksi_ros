# This message reports the baseline solution in Earth Centered Earth Fixed (ECEF) coordinates.
# This baseline is the relative vector distance from the base station to the rover receiver.
# The full GPS time is given by the preceding MSG GPS TIME with the matching time-of-week (tow).

Header header

uint32 tow
int32 x
int32 y
int32 z
uint16 accuracy
uint8 n_sats
uint8 flags
