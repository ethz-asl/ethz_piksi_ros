#! /bin/bash
# record topics necessary to debug Piksi Multi. 
# WARNING: remember to set rosparam "debug_mode" to true before launching the ROS driver!
INITIAL_PATH=$(pwd)
NOW="$(date +"%F-%H-%M-%S")"
FOLDER_NAME="$(date +"%Y-%m-%d")"

mkdir -p ~/bags/$FOLDER_NAME
cd ~/bags/$FOLDER_NAME

echo " ========================= "
echo "Record topics necessary to debug Piksi Multi. Rosbag saved in '~/bags/<DATE_OF_TODAY>/'"
echo "WARNING: remember to set rosparam 'debug_mode' to true before launching the ROS driver!"
echo " ========================= "

rosparam dump piksi-multi-debug-"$NOW".yaml

rosbag record --output-name=piksi-multi-debug-"$NOW".bag \
-e "/piksi/(.*)" \

cd $INITIAL_PATH
