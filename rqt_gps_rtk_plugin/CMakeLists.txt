cmake_minimum_required(VERSION 2.8.3)
project(rqt_gps_rtk_plugin)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
   any_worker
   rqt_gui
   rqt_gui_cpp
   piksi_rtk_msgs
   sensor_msgs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

###########
# sources #
###########
set(rqt_gps_rtk_plugin_SRCS
  src/rqt_gps_rtk_plugin/GpsRtkPlugin.cpp
)
###########



###########
# headers #
###########
set(rqt_gps_rtk_plugin_HDRS
    include/rqt_gps_rtk_plugin/GpsRtkPlugin.hpp
)
###########



############
# ui files #
############
set(rqt_gps_rtk_plugin_UIS
    resource/gps_rtk_plugin.ui
)
############

set(rqt_gps_rtk_plugin_INCLUDE_DIRECTORIES
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    any_worker
    rqt_gui
    rqt_gui_cpp
    piksi_rtk_msgs
    sensor_msgs
    DEPENDS
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_gps_rtk_plugin_MOCS ${rqt_gps_rtk_plugin_HDRS})
  qt5_wrap_ui(rqt_gps_rtk_plugin_UIS_H ${rqt_gps_rtk_plugin_UIS})
else()
  qt4_wrap_cpp(rqt_gps_rtk_plugin_MOCS ${rqt_gps_rtk_plugin_HDRS})
  qt4_wrap_ui(rqt_gps_rtk_plugin_UIS_H ${rqt_gps_rtk_plugin_UIS})
endif()

include_directories(
    ${rqt_gps_rtk_plugin_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
)


add_library(rqt_gps_rtk_plugin
    ${rqt_gps_rtk_plugin_SRCS}
    ${rqt_gps_rtk_plugin_MOCS}
    ${rqt_gps_rtk_plugin_UIS_H}
)

target_link_libraries(rqt_gps_rtk_plugin
    ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()


find_package(class_loader)
class_loader_hide_library_symbols(rqt_gps_rtk_plugin)


install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rqt_gps_rtk_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_gps_rtk_plugin
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
