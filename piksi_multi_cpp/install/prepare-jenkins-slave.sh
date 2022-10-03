#!/bin/bash -e
echo "Running the prepare script for piksi_multi_cpp.";
ROS_VERSION=`rosversion -d`
echo "ROS version: ${ROS_VERSION}"

# Build dependencies.
sudo apt install -y python3-wstool python3-catkin-tools

# Package dependencies.
echo "Installing libsbp_ros_msgs dependencies."
sudo apt install -y python3-pip libeigen3-dev libgoogle-glog-dev libgtest-dev libgeographic-dev
sudo apt install -y ros-${ROS_VERSION}-genmsg
pip3 install jinja2
pip3 install setuptools
pip3 install voluptuous

echo "Installing piksi_rtk_msgs dependencies."
sudo apt install ros-${ROS_VERSION}-rospy \
ros-${ROS_VERSION}-geometry-msgs \
ros-${ROS_VERSION}-sensor-msgs -y

echo "Installing rqt_gps_rtk_plugin dependencies."
sudo apt install ros-${ROS_VERSION}-rqt-gui ros-${ROS_VERSION}-rqt-gui-cpp qt5-default -y

echo "Installing piksi_multi_cpp dependencies."
sudo apt install ros-${ROS_VERSION}-eigen-conversions -y
sudo apt install libserialport-dev libtool -y
sudo apt install ros-${ROS_VERSION}-rosserial-server -y

echo "Installing glog_catkin dependencies."
sudo apt install autoconf -y

echo "Installing piksi_multi_interface dependencies."
sudo apt install gpiod libgpiod-dev -y
sudo apt install ros-${ROS_VERSION}-rosserial-arduino -y
sudo apt install ros-${ROS_VERSION}-tf2-eigen -y
