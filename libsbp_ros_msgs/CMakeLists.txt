cmake_minimum_required(VERSION 2.8.3)
project(libsbp_ros_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)
add_definitions(-DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

# Workaround to find GeographicLib. With Ubuntu 22.04+ should use GDAL.
# https://stackoverflow.com/questions/48169653/finding-geographiclib-in-cmake-on-debian
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

###################
## AUTOGENERATION #
###################
# Manually set Python path in case workspace is not sourced, yet.
set(ENV{PYTHONPATH} "$ENV{PYTHONPATH}:${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}")
execute_process(
  COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/msg
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/include/conversion.h
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/include/sbp_relays.h
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/src/conversion.cc
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/generator ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} --verbose
)

catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/conversion.cc
  src/geo_conversion.cc
  src/ros_conversion.cc
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${GeographicLib_LIBRARIES})

#########
# TESTS #
#########
catkin_add_gtest(test_ros_conversion
  test/ros-conversion-test.cpp
)
target_link_libraries(test_ros_conversion ${PROJECT_NAME})

catkin_add_gtest(test_geo_conversion
        test/geo-conversion-test.cpp
        )
target_link_libraries(test_geo_conversion ${PROJECT_NAME})

cs_install()
cs_export()
