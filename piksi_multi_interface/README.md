# Piksi Multi Interface
This is a push button and status LED interface for the Piksi GNSS receiver.
It simplifies utilization of RTK GNSS in the field.
It provides
- visual feedback on GNSS fix status
- push button interface to start surveying a base station or survey point position

# Setup
We setup two almost identical hardware systems to resemble a base station and a survey station.
Both stations have
- Flyht Pro Rack Tray 6U 9,5"
- Battery or shore power supply
- Antenna drawer
- Swift L1/L2 GPS/GLONASS/BeiDou mini-survey antenna
- Exposed USB ports for data logging or sensor connection
- Survey button
- GNSS status LED
- Exposed GNSS SMA port
- Exposed 868MHz modem antennas

## Base Station
Additionally, the base station spans a 2.4 and 5 GHz WiFi network and exposes
- WiFi and mobile internet antennas
- ethernet port

Its internals contain from left to right
- BOB-11189 RS232<->UART transceiver
- DELL Gateway 3001 (Ubuntu Server 18.04, 32GB recommended) (24V)
- Arduino Micro (12V)
- Adafruit 757 PPS logic level translator
- PV1F240SS survey button (5V)
- Neopixel Jewel status LED (5V)
- 5V, 12V, 24V power distribution board
- PLA50F-24 power supply
- [RFD 868x Modem](https://github.com/ethz-asl/ethz_piksi_ros/wiki/RFD-868x-Modem) on RFD900 to RS232 Interface board
- Piksi DURO (12V)
- RUTX11 mobile router (24V)

## Survey Station
The survey station does not have a router and is connected to the base station network through WiFi.

Its internals contain from left to right
- 4dBi WRT004ANT
- BOB-11189 RS232<->UART transceiver
- DELL Gateway 3001 (Ubuntu Server 18.04, 32GB recommended) (24V)
- Arduino Micro (12V)
- PV1F240SS survey button (5V)
- Adafruit 757 PPS logic level translator
- Neopixel Jewel status LED (5V)
- 5V, 12V, 24V power distribution board
- PLA50F-24 power supply
- [RFD 868x Modem](https://github.com/ethz-asl/ethz_piksi_ros/wiki/RFD-868x-Modem) on RFD900 to RS232 Interface board
- Piksi DURO (12V)

The survey antenna is mounted on a
- Leica GLS12 2m pole
- Leica GAD105 adapter

## Connectivity
All devices, except the 868MHz modem, are directly powered through the power board.
The 868MHz modem is powered through the 8 pin Piksi DURO serial connector.

The Piksi DURO connects through ethernet to the router and through UART0 to the 868MHz modem. Optionally, the AUX connector connects the PPS signal through 757 to pin 2 GPIO of the DELL gateway and the UART1 NMEA timing signal to the second RS232 port.

The Arduino Micro pin 6 connects to the Neopixel data input.
Its serial connection is connected to the Dell's first RS232 port via BOB-11189.

The PV1F240SS push button connect 5V to Dell's pin 1 GPIO.

# Usage
Both base and survey station have a Linux `piksi` and `piksi_interface` service enabled.
This will automatically start the `piksi_multi_cpp` driver, `pushbutton_node`, and `rosserial` connection to the Arduino on startup.

Once the receivers obtain a GNSS fix punch the survey button to start sampling the antenna position.
By default the base station will average `1000` samples, the survey station `100` samples.
The survey station subtracts 2 meters from the surveyed position to account for the survey stick.
The surveyed position is saved in the `ROS_HOME` directory.


| LED | Description          | Status                                                                                        |
| ------------- | ------------- |------------------------------------------------------------------------------------------- |
|  | Rainbow                 | Rosserial is not connected. ~3 minutes at startup.                                            |
|  | Red blinking ring       | No GNSS fix. Antenna connected?                                                               |
|  | Yellow ring             | SPP fix, 3 sats per LED                                                                       |
|  | Green ring              | SPP fix, 3 sats per LED                                                                       |
|  | Blue blinking ring      | RTK float, 3 sats per LED                                                                     |
|  | Blue ring               | RTK fix, 3 sats per LED                                                                       |
|  | Red blinking center     | Receiving RTK corrections                                                                     |
|  | Magenta blinking center | Surveying position, blinking frequency increases with advanced progress. Stops when finished. |

# Installation
The system requires [piksi_multi_cpp](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_cpp) and its [autostart](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_multi_cpp/install/install_autostart.sh) feature.
The [interface installation script](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_multi_interface/install/install.sh) describes the setup procedure.
The [interface installation script](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/firmware/setup.sh) installs the Arduino firmware via command line.

Optionally, [PPS sync installation script](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_pps_sync/install/install.sh) will setup chrony to synchronize the system clock with GNSS time.
