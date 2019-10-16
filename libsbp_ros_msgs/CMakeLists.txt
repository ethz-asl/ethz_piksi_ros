cmake_minimum_required(VERSION 2.8.3)
project(libsbp_ros_msgs)

add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
#  src/conversion.cc
#)
#target_link_libraries(${PROJECT_NAME})

catkin_python_setup()
catkin_install_python(PROGRAMS bin/generator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()
cs_export()
