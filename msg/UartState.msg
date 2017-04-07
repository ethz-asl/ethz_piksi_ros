# The UART message reports data latency and throughput of the UART channels providing SBP I/O.
# On the default Piksi configuration, UARTs A and B are used for telemetry radios, but can also be host
# access ports for embedded hosts, or other interfaces in future. The reported percentage values must
# be normalized. Observations latency and period can be used to assess the health of the differential
# corrections link. Latency provides the timeliness of received base observations while the period indicates
# their likelihood of transmission.

Header header

float64 uart_a_tx_throughput
float64 uart_a_rx_throughput
uint16 uart_a_crc_error_count
uint16 uart_a_io_error_count
uint8 uart_a_tx_buffer_level
uint8 uart_a_rx_buffer_level

float64 uart_b_tx_throughput
float64 uart_b_rx_throughput
uint16 uart_b_crc_error_count
uint16 uart_b_io_error_count
uint8 uart_b_tx_buffer_level
uint8 uart_b_rx_buffer_level

int32 latency_avg
int32 latency_lmin
int32 latency_lmax
int32 latency_current
