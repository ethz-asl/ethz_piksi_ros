cmake_minimum_required(VERSION 2.8.3)
project(libsbp_ros_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)
add_definitions(-DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)

find_package(catkin_simple REQUIRED)
find_package (Eigen3 REQUIRED)

###################
## AUTOGENERATION #
###################
# REQUIRES CATKIN MERGE DEVEL SPACE
catkin_python_setup() # Must be called before generate_messages (inside catkin_simple) and must be called before running rosrun libsbp_ros_msgs generator
execute_process(
  COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/msg
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/include/conversion.h
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/include/sbp_relays.h
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/src/conversion.cc
  COMMAND rosrun libsbp_ros_msgs generator --verbose
)

catkin_simple(ALL_DEPS_REQUIRED)
#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/conversion.cc
  src/ros_conversion.cc
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

catkin_install_python(PROGRAMS bin/generator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
cs_install()
cs_export()
