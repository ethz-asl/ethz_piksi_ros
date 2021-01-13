cmake_minimum_required(VERSION 2.8.3)
project(libsbp_ros_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)
add_definitions(-DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)

find_package(catkin REQUIRED COMPONENTS
             libsbp_catkin
             rospack
             message_generation)
find_package(Eigen3 REQUIRED)

###################
## AUTOGENERATION #
###################
# REQUIRES CATKIN MERGE DEVEL SPACE
execute_process(
  COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/msg
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/include/conversion.h
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/include/sbp_relays.h
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/src/conversion.cc
  COMMAND python2.7 ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/generator --verbose
)
add_message_files(DIRECTORY msg)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS libsbp_catkin rospack message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/conversion.cc
  src/ros_conversion.cc
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

#########
# TESTS #
#########
catkin_add_gtest(test_ros_conversion
  test/ros-conversion-test.cpp
)
target_link_libraries(test_ros_conversion ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION include
)

catkin_package(CATKIN_DEPENDS message_runtime)
