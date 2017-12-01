piksi_rtk_gps
======
**WARNING: this ROS driver for Piksi V2.3 is NOT longer supported!**

**If you have a Piksi Multi, please check the lates ROS driver: [piksi_multi_rtk_ros](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros)**

ROS node to read SBP messages from an attached Piksi **V2** RTK device.

Based on the work of Daniel Eckert: [Original repo](https://bitbucket.org/Daniel-Eckert/mav_localization).

## Installation and Configuration

### Dependencies
  * python-pip `sudo apt-get install python-pip`
  * python-tox `sudo apt-get install python-tox`
  * pandoc     `sudo apt-get install pandoc`

### Installation
**WARNING: install __ONLY ONE__ version of SBP library, depending of which Hardware version you are using. This page cointains the driver for [Piksi V2.3](http://docs.swiftnav.com/pdfs/piksi_datasheet_v2.3.1.pdf). If you are using [Piksi Multi](https://www.swiftnav.com/piksi-multi) please check its driver version: [piksi_multi_rtk_ros](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros).**

The following code will automatically download the required version of libsbp and install it in the default folder `/usr/local/lib/python2.7/dist-packages/sbp-1.2.1-py2.7.egg/sbp/`.

```
#from the repository folder
chmod +x install/install_piksi_v2.sh  #make sure the script is excutable
./install/install_piksi_v2.sh         # WARNING, this script will:
                                      # 1. Create udev rule for Piksi
                                      # 2. Add you to dialout, if you are not within this group
```

## Usage
Make sure you configured your Piksi(s) by following [these instructions](https://github.com/ethz-asl/mav_rtk_gps/wiki/Installing-and-Configuring-Piksi#settings-piksi-v2).

**Base station**
```
roslaunch piksi_rtk_gps piksi_multi_base_station.launch
```

**MAV**
```
roslaunch piksi_rtk_gps piksi_multi_mav.launch
```
Check the status of your receiver:
```
roslaunch mav_rtk_gui rtk_info_example.launch
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
 - `base_station_ip_for_latency_estimation: <IP address of base station or router, to estimate latency over wifi>`
