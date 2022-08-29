#!/bin/bash
source /home/$USER/catkin_ws/devel/setup.bash
roslaunch piksi_multi_interface interface_rpi.launch ns:=$1 &
