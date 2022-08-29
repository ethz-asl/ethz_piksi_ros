#!/bin/bash
source /home/$USER/catkin_ws/devel/setup.bash
roslaunch piksi_multi_interface interface.launch chip:=$1 offset_pushbutton:=$2 port:=$3 baud:=$4 ns:=$5 &
