cmake_minimum_required(VERSION 2.8.3)
project(piksi_status_indicator)

find_package(catkin REQUIRED
  COMPONENTS
    piksi_rtk_msgs
    neostate
    rospy
)

catkin_package(
  CATKIN_DEPENDS
    piksi_rtk_msgs
    neostate
    rospy
)

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/start_status_indicator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
