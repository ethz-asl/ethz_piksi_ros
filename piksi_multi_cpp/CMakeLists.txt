cmake_minimum_required(VERSION 2.8.3)
project(piksi_multi_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/device/device.cc
        src/device/device_factory.cc
        src/device/device_usb.cc
        src/device/device_serial.cc
        src/device/device_tcp.cc
        src/observations/udp_observation_receiver.cc
        src/observations/cb_to_raw_obs_converter.cc
        src/observations/udp_observation_sender.cc
        src/observations/file_observation_logger.cc
        src/receiver/receiver_attitude.cc
        src/receiver/receiver_base_station.cc
        src/receiver/receiver_factory.cc
        src/receiver/receiver_position.cc
        src/receiver/receiver_ros.cc
        src/receiver/receiver.cc
        src/receiver/settings_io.cc
        src/sbp_callback_handler/sbp_callback_handler_relay/ros_base_pos_relay.cc
        src/sbp_callback_handler/sbp_callback_handler_relay/ros_relays.cc
        src/sbp_callback_handler/sbp_callback_handler_relay/ros_imu_relay.cc
        src/sbp_callback_handler/sbp_callback_handler_relay/ros_enu_relays.cc
        src/sbp_callback_handler/sbp_callback_handler_relay/ros_mag_relay.cc
        src/sbp_callback_handler/sbp_callback_handler_relay/ros_receiver_state.cc
        src/sbp_callback_handler/sbp_callback_handler_relay/ros_time_ref_relay.cc
        src/sbp_callback_handler/geotf_handler.cc
        src/sbp_callback_handler/position_sampler.cc
        src/sbp_callback_handler/ros_time_handler.cc
        src/sbp_callback_handler/sbp_callback_handler.cc
        src/sbp_callback_handler/sbp_callback_handler_factory.cc
        src/sbp_callback_handler/sbp_observation_callback_handler.cc
        )
target_link_libraries(${PROJECT_NAME} serialport stdc++fs)

############
# BINARIES #
############
cs_add_executable(piksi_multi src/piksi_multi_node.cc)
target_link_libraries(piksi_multi ${PROJECT_NAME})

cs_add_executable(piksi_multi_config src/piksi_multi_config_node.cc)
target_link_libraries(piksi_multi_config ${PROJECT_NAME})

cs_install()
cs_export()
