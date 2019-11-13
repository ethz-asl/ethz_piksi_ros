#!/bin/bash -e
echo "Running the prepare script for piksi_multi_cpp.";
ROS_VERSION=`rosversion -d`
echo "ROS version: ${ROS_VERSION}"

# Build dependencies.
sudo apt-get install -y python-wstool python-catkin-tools

# Package dependencies.
echo "Installing libsbp_ros_msgs dependencies."
sudo apt install -y python-pip
pip install jinja2
pip install setuptools
pip install voluptuous

echo "Installing geodetic_utils and geotf dependencies."
sudo apt install ros-${ROS_VERSION}-tf-conversions -y
sudo apt install libgdal-dev -y

echo "Installing rqt_gps_rtk_plugin dependencies."
sudo apt install ros-${ROS_VERSION}-rqt-gui ros-${ROS_VERSION}-rqt-gui-cpp qt5-default -y
