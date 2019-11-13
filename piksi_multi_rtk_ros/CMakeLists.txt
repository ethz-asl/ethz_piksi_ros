cmake_minimum_required(VERSION 2.8.3)
project(piksi_multi_rtk_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    visualization_msgs
    piksi_rtk_msgs
    sensor_msgs
    roslib
    rospy
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    visualization_msgs
    sensor_msgs
    piksi_rtk_msgs
    roslib
    rospy
#  DEPENDS
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#############
## Install ##
#############
 catkin_install_python(PROGRAMS bin/piksi_multi bin/geodetic_survey
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY cfg launch log_surveys rosbag_record_scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
