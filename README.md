piksi_rtk_gps
======
ROS node to read SBP messages from an attached Piksi **V2** RTK device.

Based on the work of Daniel Eckert: [Original repo](https://bitbucket.org/Daniel-Eckert/mav_localization).
  
## Installation and Configuration

### Dependencies
  * libsbp (tested wit: [libsbp 1.2.1](https://github.com/swift-nav/libsbp/tree/v1.2.1))
  * python-pip `apt-get install python-pip`
  * python-tox `apt-get install python-tox`
  * pandoc     `apt-get install pandoc`
  
### Installation
The following code will automatically download the required version of libsbp and install it in the default folder `/usr/local/lib/python2.7/dist-packages/sbp-VERSION_NUMBER-py2.7.egg/sbp/`.

```
#from the repository folder
chmod +x install/install_piksi.sh  #make sure the script is excutable
./install/install_piksi_v2.sh         # WARNING, this script will:
                                      # 1. Create udev rule for Piksi
                                      # 2. Add you to dialout, if you are not within this group
```
## Corrections Over Wifi
It is possible to send/receive corrections over Wifi between multiple Piksi modules.
Set configurations in `cfg/piksi_driver_settings.yaml`
- Base station configuration (sends corrections)
 - `base_station_mode: true`
 - `broadcast_addr: <put here Bcast address obtained from command ifconfig>`
 - `broadcast_port: 26078`
- Rover configuration (receives corrections)
 - `base_station_mode: false`
 - `broadcast_addr: <put here Bcast address obtained from command ifconfig>`
 - `broadcast_port: 26078`
