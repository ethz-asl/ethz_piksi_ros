cmake_minimum_required(VERSION 2.8.3)
project(piksi_v2_rtk_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    rospy
    piksi_rtk_msgs
    sensor_msgs
    geometry_msgs
)

# Deprecate this package
set(CMAKE_WARN_DEPRECATED TRUE)
message(DEPRECATION "${PROJECT_NAME} is deprecated: please switch to Piksi Multi.")

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
    piksi_rtk_msgs
    sensor_msgs
    geometry_msgs
#  DEPENDS
)
