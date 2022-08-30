cmake_minimum_required(VERSION 2.8.3)
project(piksi_multi_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_library(GPIOD_LIBRARY NAMES libgpiod.so)
if(NOT GPIOD_LIBRARY)
  message(FATAL_ERROR "gpiod library not found. Install https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/")
endif()

############
# BINARIES #
############
cs_add_executable(pushbutton src/pushbutton_node.cc)
target_link_libraries(pushbutton ${GPIOD_LIBRARY})

cs_add_executable(survey_status src/survey_status_node.cc)
target_link_libraries(survey_status ${GPIOD_LIBRARY})

catkin_install_python(PROGRAMS scripts/neopixel_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

cs_install()
cs_export()
