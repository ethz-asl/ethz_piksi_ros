# This message reports the baseline solution in North East Down (NED) coordinates. This baseline is
# the relative vector distance from the base station to the rover receiver, and NED coordinate system is
# defined at the local WGS84 tangent plane centered at the base station position. The full GPS time is
# given by the preceding MSG GPS TIME with the matching time-of-week (tow).

Header header

uint32 tow
int32 n
int32 e
int32 d
uint16 h_accuracy
uint16 v_accuracy
uint8 n_sats
uint8 flags
