source ~/catkin_ws/devel/setup.bash
rm -rf libraries/ros_lib
rosrun rosserial_arduino make_libraries.py libraries

SKETCHBOOK_PATH=/home/$USER/catkin_ws/src/ethz_piksi_ros/piksi_multi_interface/firmware

arduino --install-boards arduino:avr
arduino --pref "sketchbook.path=$SKETCHBOOK_PATH" --install-library Adafruit\ NeoPixel:1.4.0
arduino --pref "sketchbook.path=$SKETCHBOOK_PATH" --upload --board arduino:avr:micro --port /dev/ttyACM0 status_led/status_led.ino
