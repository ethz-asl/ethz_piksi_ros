cmake_minimum_required(VERSION 2.8.3)
project(piksi_multi_msgs)

add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/conversions.cc
)
target_link_libraries(${PROJECT_NAME})

cs_install()
cs_export()
