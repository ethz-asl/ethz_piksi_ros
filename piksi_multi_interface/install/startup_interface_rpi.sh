#!/bin/bash
source /home/$2/catkin_ws/devel/setup.bash
roslaunch piksi_multi_interface interface_rpi.launch ns:=$1 &
