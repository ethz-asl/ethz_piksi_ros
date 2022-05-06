# Installation
Create a catkin workspace on [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).
```
sudo apt install -y python3-wstool python3-catkin-tools
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config --extend /opt/ros/noetic
```

Download this package.
```
cd ~/catkin_ws/src
wstool init
wstool set --git ethz_piksi_ros git@github.com:ethz-asl/ethz_piksi_ros.git
wstool update
```

Install all [PPA dependencies](install/prepare-jenkins-slave.sh).
```
source /opt/ros/noetic/setup.bash
./ethz_piksi_ros/piksi_multi_cpp/install/prepare-jenkins-slave.sh
```

Next download all individual ROS package dependencies.
**Note**: If you have not setup [SSH keys in GitHub](https://help.github.com/en/enterprise/2.16/user/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) use [dependencies_https.rosinstall](install/dependencies_https.rosinstall).
```
wstool merge ethz_piksi_ros/piksi_multi_cpp/install/dependencies.rosinstall
wstool update -j8
```

Finally, build the workspace.
```
catkin build
```

# Basic usage Base-Rover
1. [Upgrade firmware](https://support.swiftnav.com/support/solutions/articles/44001903720-upgrading-firmware) on your Piksi Multi to **2.4.16** 
2. Flash rover or base configuration onto your Piksi Multi
    1. Hookup Piksi Multi onto host computer via USB (you can also specify another [interface here](launch/config.launch#L3)).
    2. `roslaunch piksi_multi_cpp config.launch config_type:=rover` or<br>`roslaunch piksi_multi_cpp config.launch config_type:=base`
3. Setup the correction link from base to rover either via UDP broadcast and/or serial modem.
    1. Specify [UDP broadcast interface](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_multi_cpp/launch/base.launch#L13) if you want to send corrections via network.
    2. Setup base and rover [serial modem](https://github.com/ethz-asl/ethz_piksi_ros/wiki/RFD-868x-Modem).
4. Launch driver on base `roslaunch piksi_multi_cpp base.launch`
5. Sample base station position once
    1. ```rosservice call /piksi_multi_cpp_base/base_station_receiver_0/resample_base_position "num_desired_fixes: 1000 file: '/tmp/base_position.txt' set_enu: false offset_z: 0.0"```
    2. Check status of sampling <br>`rostopic echo /piksi_multi_cpp_base/base_station_receiver_0/position_sampler/position_sampling`
6. **After base station sampling** you can start the driver on the rover `roslaunch piksi_multi_cpp rover.launch`
