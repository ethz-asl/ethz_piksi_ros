std_msgs/Float32 tx_throughput    # UART transmit throughput
std_msgs/Float32 rx_throughput    # UART receive throughput
std_msgs/UInt16  crc_error_count  # UART CRC error count
std_msgs/UInt16  io_error_count   # UART IO error count
std_msgs/UInt8   tx_buffer_level  # UART transmit buffer percentage utilization (ranges from 0 to 255)
std_msgs/UInt8   rx_buffer_level  # UART receiver buffer percentage utilization (ranges from 0 to 255)
