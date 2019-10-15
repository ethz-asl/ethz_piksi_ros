# MSG SBAS RAW — 0x7777 — 30583

SatelliteIdentifier sid
uint32              tow          # GPS Time of Week
uint8               message_type # SBAS message type (0-63)
uint8[27]           data         # Raw SBAS data field of 212 bits (last byte padded with zeros).
