# Piksi Multi Interface
This is a push button and status LED interface for the Piksi GNSS receiver.
It simplifies utilization of RTK GNSS in the field.
It provides
- WiFi network with internet access
- push button interface to start surveying a base station or survey GNSS positions
- visual feedback on GNSS fix status
- GNSS synchronized gateway to record USB or ethernet connected sensors, e.g., total station

![Hardware overview](https://user-images.githubusercontent.com/11293852/84021463-ef130600-a984-11ea-86f0-79b5327cb64e.jpg)

# Usage
Both base and survey station have a Linux `piksi` and `piksi_interface` service enabled.
This will automatically start the `piksi_multi_cpp` driver, `pushbutton_node`, and `rosserial` connection to the Arduino on startup.

Once the receivers obtain a GNSS fix punch the survey button to start sampling the antenna position.
By default the base station will average `1000` samples, the survey station `100` samples.
The survey station subtracts 2 meters from the surveyed position to account for the survey pole.
The surveyed position is saved in the `ROS_HOME` directory.


| LED | Description          | Status                                                                                        |
| ------------- | ------------- |------------------------------------------------------------------------------------------- |
| <img src="https://user-images.githubusercontent.com/11293852/84021852-a3149100-a985-11ea-8df5-a5966ede3669.jpg" alt="Status not connected" width="220"/> | Rainbow                 | Rosserial is not connected. ~3 minutes at startup.                                            |
| <img src="https://user-images.githubusercontent.com/11293852/84021856-a4de5480-a985-11ea-902b-dbf3f536aca0.jpg" alt="Status no fix" width="220"/> | Red blinking ring       | No GNSS fix. Antenna connected?                                                               |
| <img src="https://user-images.githubusercontent.com/11293852/84021871-a7d94500-a985-11ea-84e5-3c6060723b0f.jpg" alt="Status SPP fix" width="220"/> | Yellow ring             | SPP fix, 3 sats per LED                                                                       |
| <img src="https://user-images.githubusercontent.com/11293852/84022240-38b02080-a986-11ea-9c7a-ac73fb20af38.jpg" alt="Status SBAS fix" width="220"/> | Green ring              | SPP fix, 3 sats per LED                                                                       |
| <img src="https://user-images.githubusercontent.com/11293852/84039501-9bfb7c00-a9a1-11ea-8115-e8cc98ea5b5e.jpg" alt="Status DGNSS" width="220"/> | Purple ring      | DGNSS, 3 sats per LED                                                                     |
| <img src="https://user-images.githubusercontent.com/11293852/84022299-4fef0e00-a986-11ea-8336-f2fcbb153993.jpg" alt="Status RTK float" width="220"/> | Blue blinking ring      | RTK float, 3 sats per LED                                                                     |
| <img src="https://user-images.githubusercontent.com/11293852/84022299-4fef0e00-a986-11ea-8336-f2fcbb153993.jpg" alt="Status RTK fix" width="220"/> | Blue ring               | RTK fix, 3 sats per LED                                                                       |
| <img src="https://user-images.githubusercontent.com/11293852/84021875-a9a30880-a985-11ea-8a04-82e70a3b2b75.jpg" alt="Status corrections" width="220"/> | Red blinking center     | Receiving RTK corrections                                                                     |
| <img src="https://user-images.githubusercontent.com/11293852/84021860-a60f8180-a985-11ea-9ae0-5dd12eb4bf0d.jpg" alt="Status sampling" width="220"/> | Magenta blinking center | Surveying position, blinking frequency increases with advanced progress. Stops when finished. |

# Hardware Setup
We setup two almost identical hardware systems to resemble a base station and a survey station.
Both stations have
- Flyht Pro Rack 6U 9,5"
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
<img src="https://user-images.githubusercontent.com/11293852/84021399-cc80ed00-a984-11ea-9ab7-7fb420d9b92a.jpg" alt="Base station front" width="480"/>

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
<img src="https://user-images.githubusercontent.com/11293852/84021373-c12dc180-a984-11ea-8692-61ff5744580b.jpg" alt="Survey station internal" width="680"/>

## Survey Station
The survey station does not have a router and is connected to the base station network through WiFi.
<img src="https://user-images.githubusercontent.com/11293852/84021387-c854cf80-a984-11ea-898c-2a290b8ce182.jpg" alt="Survey station front" width="480"/>

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
<img src="https://user-images.githubusercontent.com/11293852/84021383-c68b0c00-a984-11ea-941b-1c00c962b914.jpg" alt="Survey station internal" width="680"/>

The survey antenna is mounted on a
- Leica GLS12 2m pole
- Leica GAD105 adapter
<img src="https://user-images.githubusercontent.com/11293852/84022035-f1c22b00-a985-11ea-8d98-b293cc7628df.jpg" alt="Survey pole" width="680"/>

## Connectivity
All devices, except the 868MHz modem, are directly powered through the power board.
The 868MHz modem is powered through the 8 pin Piksi DURO serial connector.

The Piksi DURO connects through ethernet to the router and through UART0 to the 868MHz modem. Optionally, the AUX connector connects the PPS signal through 757 to pin 2 GPIO of the DELL gateway and the UART1 NMEA timing signal to the second RS232 port.

The Arduino Micro pin 6 connects to the Neopixel data input.
Its serial connection is connected to the Dell's first RS232 port via BOB-11189.

The PV1F240SS push button connect 5V to Dell's pin 1 GPIO.

# Installation
The system requires [piksi_multi_cpp](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_cpp) and its [autostart](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_multi_cpp/install/install_autostart.sh) feature.
The [interface installation script](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_multi_interface/install/install.sh) describes the setup procedure.
The [interface installation script](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/firmware/setup.sh) installs the Arduino firmware via command line.

Optionally, [PPS sync installation script](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_pps_sync/install/install.sh) will setup chrony to synchronize the system clock with GNSS time.

# Acknowledgement
The development of this robotic base station was kindly supported by [Project FindMine](https://www.ue-stiftung.org/findmine) and the Urs Endress Foundation.

# Docker
```
docker run -it --network host --privileged --env ROS_MASTER_URI=http://$HOSTNAME:11311 ros:noetic
``

```
docker exec -it modest_lederberg bash
```