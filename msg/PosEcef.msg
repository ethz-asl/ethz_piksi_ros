# The position solution message reports absolute Earth Centered Earth Fixed (ECEF) coordinates and the
# status (single point vs pseudo-absolute RTK) of the position solution. If the rover receiver knows the
# surveyed position of the base station and has an RTK solution, this reports a pseudo-absolute position
# solution using the base station position and the rover’s RTK baseline vector. The full GPS time is given
# by the preceding MSG GPS TIME with the matching time-of-week (tow).

Header header

uint32 tow
float64 x
float64 y
float64 z
uint16 accuracy
uint8 n_sats
uint8 flags
