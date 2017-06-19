piksi_rtk_gps
======
ROS node to read SBP messages from an attached Piksi **Multi** RTK device.
  
## Installation and Configuration

### Dependencies
  * libsbp (tested wit: [libsbp 2.2.1](https://github.com/swift-nav/libsbp/tree/v2.2.1))
  * python-pip `apt-get install python-pip`
  * python-tox `apt-get install python-tox`
  * pandoc     `apt-get install pandoc`
  
### Installation
**WARNING: install __ONLY ONE__ version of SBP library, depending of which Hardware version you are using. This page cointains the driver for [Piksi Multi](https://www.swiftnav.com/piksi-multi). If you are using [Piksi V2](http://docs.swiftnav.com/pdfs/piksi_datasheet_v2.3.1.pdf) please check its driver version: [piksi_rtk_gps](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_gps)**

The following code will automatically download the required version of libsbp and install it in the default folder `/usr/local/lib/python2.7/dist-packages/sbp-2.2.1-py2.7.egg/sbp/`.

```
#from the repository folder
chmod +x install/install_piksi.sh  #make sure the script is excutable
./install/install_piksi_multi.sh         # WARNING, this script will:
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
