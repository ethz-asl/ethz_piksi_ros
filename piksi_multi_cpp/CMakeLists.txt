cmake_minimum_required(VERSION 2.8.3)
project(piksi_multi_cpp)

add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/device.cc
  src/device_usb.cc
  src/piksi_multi.cc
  src/ros_attitude_receiver.cc
  src/ros_base_station_receiver.cc
  src/ros_output.cc
  src/ros_output_heartbeat.cc
  src/ros_position_receiver.cc
  src/ros_receiver.cc
)
target_link_libraries(${PROJECT_NAME} serialport)

############
# BINARIES #
############
cs_add_executable(piksi_multi src/piksi_multi_node.cc)
target_link_libraries(piksi_multi ${PROJECT_NAME})

cs_install()
cs_export()
