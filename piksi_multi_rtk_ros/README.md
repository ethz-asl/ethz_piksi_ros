piksi_multi_rtk_gps
======
ROS node to read SBP messages from an attached Piksi **Multi** RTK device.

The piksi_multi_rtk_ros package has been tested under ROS Melodic and Ubuntu 18.04.


**WARNING:** default baud rate of the driver is set to '230400' (default baud rate of Piksi Multi is '115200').
This means you have to set your Piksi Multi baud rate correctly by following [these settings instructions](https://github.com/ethz-asl/ethz_piksi_ros/wiki/Installing-and-Configuring-Your-Piksi-Multi#settings).

## Installation and Configuration
**WARNING: install __ONLY ONE__ version of SBP library, depending of which Hardware version you are using. This page contains the driver for [Piksi Multi](https://www.swiftnav.com/piksi-multi).
If you are using [Piksi V2](http://docs.swiftnav.com/pdfs/piksi_datasheet_v2.3.1.pdf) please check its driver version: [piksi_rtk_gps](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_gps)** (it is not supported anymore).

The following code will automatically download the required version of libsbp and install it in the default folder `/usr/local/lib/python2.7/dist-packages/sbp-<SBP_LIB_VERSION>-py2.7.egg/sbp/`.

```
# Execute this line in the package folder 'piksi_multi_rtk_ros'
source install/install_piksi_multi.sh
```

### Firmware and SBP Lib Version
Please check [here](https://support.swiftnav.com/customer/en/portal/articles/2492810-swift-binary-protocol) which Piksi Multi firmware version based on the current SBP Lib version.

Currently the `install_piksi_multi.sh` will install **SBP Lib 2.6.5** (see [REPO_TAG](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_multi_rtk_ros/install/install_piksi_multi.sh#L4)).
This means you are supposed to install **Firmware 2.3.19** from [SwiftNav Firmware page](https://support.swiftnav.com/customer/en/portal/articles/2492784-piksi-multi-and-duro-firmware) in your Piksi Multi.
**WARNING: If upgrading from a firmware below v2.0.0 to a firmware greater than v2.0.0, you must upgrade to v2.0.0 first.**

## Usage
**Make sure** you configured your Piksi(s) by following [these instructions](https://github.com/ethz-asl/ethz_piksi_ros/wiki/Installing-and-Configuring-Your-Piksi).

**Base station**

If you have already configured Piksi Multi to act as a base station and sampled its position (see [Testing Outdoor](https://github.com/ethz-asl/ethz_piksi_ros/wiki/Testing-Outdoors)), then simply launch the following file.
```
roslaunch piksi_multi_rtk_ros piksi_multi_base_station.launch
```
If you have already configured Piksi Multi to act as a base station (and did **NOT** sample its position), then launch the following file.
Once the survey is over, the sampled position will be automatically written in Piksi flash memory.
```
roslaunch piksi_multi_rtk_ros geodetic_survey.launch
```

**Rover**
```
roslaunch piksi_multi_rtk_ros piksi_multi_rover.launch
```
Check the status of your receiver (RQT GUI):
```
roslaunch rqt_gps_rtk_plugin gui.launch
```

## Advertised Topics
The most interesting advertised topics are:

 - `/piksi/navsatfix_rtk_fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))
   in case of RTK fix position from Piksi then this message contains WGS 84 coordinates;
 - `/piksi/navsatfix_spp` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))
   in case of SPP fix position from Piksi then this message contains WGS 84 coordinates;
 - `/piksi/navsatfix_best_fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))
   this message contains WGS 84 coordinates with best available fix at the moment (either RTK or SPP);
 - `/piksi/debug/receiver_state` ([piksi_rtk_msgs/ReceiverState_V2_2_15](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_rtk_msgs/msg/ReceiverState_V2_2_15.msg))
   this message contains miscellaneous information about the state of Piksi Multi receiver;
 - `/piksi/enu_pose_fix` ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))
   this message contains ENU (East-North-Up) coordinate of the receiver in case of RTK fix. Orientation is set to identity quaternion (w=1);
 - `/piksi/enu_pose_spp` ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))
   this message contains ENU (East-North-Up) coordinate of the receiver in case of SPP fix. Orientation is set to identity quaternion (w=1);
 - `/piksi/enu_pose_best_fix` ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))
   this message contains ENU (East-North-Up) coordinate of the receiver with best available fix at the moment (either RTK or SPP). Orientation is set to identity quaternion (w=1);

For a complete list of advertised topics please check function [`advertise_topics`](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_multi_rtk_ros/src/piksi_multi.py#L264).

### Raw IMU and Magnetometer Measurements
Raw IMU and magnetometer measurements are not published by default. If you want to publish them in ROS you need to: (a) Open Swifnav console, connect to Piksi, navigate to "Settings", and then in the "imu" section enable "imu raw output" and "mag raw output" (save these new settings, so you need to do this operation only once); (b) set to true the flag [publish_raw_imu_and_mag](./cfg/piksi_multi_driver_settings.yaml#L19).
Raw measurements are published in `/piksi/imu_raw` and `/piksi/mag_raw`.

### Position measurements with covariance information.
Set `publish_covariances: true` to publish position measurements with covariance information set.

### GPS time stamping.
`use_gps_time` enables GPS time stamped measurements. Otherwise the measurements are time stamped on arrival. This requires [synchronization against PPS](https://github.com/ethz-asl/ethz_piksi_ros/wiki/Piksi-PPS-Sync).

## Origin ENU Frame
The origin of the ENU (East-North-Up) frame is set either using ROS parameters or using the first RTK fix message obtained.
In the former case the following private parameters must be set:

 - `latitude0_deg`: latitude where the origin of ENU frame is located, in degrees;
 - `longitude0_deg`: longitude where the origin of ENU frame is located, in degrees;
 - `altitude0`: altitude where the origin of ENU frame is located, in meters;

see also configuration file [enu_origin_example.yaml](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_multi_rtk_ros/cfg/enu_origin_example.yaml).

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

## Rosservice
The following services are available:
 - `piksi/settings_write`: writes a specific setting to Piksi Multi;
 - `piksi/settings_read_req`: requests a parameter to Piksi Multi;
 - `piksi/settings_read_resp`: reads the latest response of a "settings_read_req" request;
 - `piksi/settings_save`: saves the current settings of Piksi Multi to its flash memory.
