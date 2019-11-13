cmake_minimum_required(VERSION 2.8.3)
project(piksi_pps_sync)

find_package(catkin REQUIRED
  COMPONENTS
    rosbag
    rospy
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rosbag
    rospy
    sensor_msgs
)

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/evaluate_pps
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
