piksi_multi_rtk_gps
======
ROS node to read SBP messages from an attached Piksi **Multi** RTK device.
  
## Installation and Configuration

### Dependencies
  * python-pip `sudo apt-get install python-pip`
  * python-tox `sudo apt-get install python-tox`
  * pandoc     `sudo apt-get install pandoc`
  
### Installation
**WARNING: install __ONLY ONE__ version of SBP library, depending of which Hardware version you are using. This page cointains the driver for [Piksi Multi](https://www.swiftnav.com/piksi-multi). If you are using [Piksi V2](http://docs.swiftnav.com/pdfs/piksi_datasheet_v2.3.1.pdf) please check its driver version: [piksi_rtk_gps](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_gps).**

The following code will automatically download the required version of libsbp and install it in the default folder `/usr/local/lib/python2.7/dist-packages/sbp-2.2.1-py2.7.egg/sbp/`.

```
# From the repository folder
source install/install_piksi_multi.sh
```

## Usage
Make sure you configured your Piksi(s) by following [these instructions](https://github.com/ethz-asl/mav_rtk_gps/wiki/Installing-and-Configuring-Piksi#settings-piksi-multi).

**Base station**
```
roslaunch piksi_multi_rtk_gps piksi_multi_base_station.launch
```

**MAV**
```
roslaunch piksi_multi_rtk_gps piksi_multi_mav.launch
```
Check the status of your receiver:
```
roslaunch mav_rtk_gui rtk_multi_info_example.launch
```

## Corrections Over Wifi
It is possible to send/receive corrections over Wifi between multiple Piksi modules.
Set configurations in `cfg/piksi_multi_driver_settings.yaml`
- Base station configuration (sends corrections)
 - `base_station_mode: true`
 - `broadcast_addr: <put here Bcast address obtained from command ifconfig>`
 - `broadcast_port: 26078`
- Rover configuration (receives corrections)
 - `base_station_mode: false`
 - `broadcast_addr: <put here Bcast address obtained from command ifconfig>`
 - `broadcast_port: 26078`
 - `base_station_ip_for_latency_estimation: <IP address of base station or router, to estimate latency over wifi>`
