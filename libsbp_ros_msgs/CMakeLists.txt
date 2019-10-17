cmake_minimum_required(VERSION 2.8.3)
project(libsbp_ros_msgs)

add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)

###################
## AUTOGENERATION #
###################
execute_process(
  COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/msg
  COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/include
  COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/export
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/src/conversion.cc
  COMMAND rosrun libsbp_ros_msgs generator --verbose
)

catkin_python_setup() # Must be called before generate_messages (inside catkin_simple)
catkin_simple(ALL_DEPS_REQUIRED)
#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/conversion.cc
)

catkin_install_python(PROGRAMS bin/generator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
cs_install()
cs_export()
