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
  src/device/device.cc
  src/device/device_factory.cc
  src/device/device_usb.cc
  src/receiver/receiver_attitude.cc
  src/receiver/receiver_base_station.cc
  src/receiver/receiver_factory.cc
  src/receiver/receiver_position.cc
  src/receiver/receiver.cc
  src/sbp_callback_handler/sbp_callback_handler.cc
  src/sbp_callback_handler/sbp_callback_handler_factory.cc
  src/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.cc
  src/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay_heartbeat.cc
  src/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay_imu_raw.cc
)
target_link_libraries(${PROJECT_NAME} serialport)

############
# BINARIES #
############
cs_add_executable(piksi_multi src/piksi_multi_node.cc)
target_link_libraries(piksi_multi ${PROJECT_NAME})

cs_install()
cs_export()
