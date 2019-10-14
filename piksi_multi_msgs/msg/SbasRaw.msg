# MSG SBAS RAW — 0x7777 — 30583

piksi_multi_msgs/SatelliteIdentifier sid
std_msgs/UInt32                      tow          # GPS Time of Week
std_msgs/UInt8                       message_type # SBAS message type (0-63)
std_msgs/UInt8[27]                   data         # Raw SBAS data field of 212 bits (last byte padded with zeros).
