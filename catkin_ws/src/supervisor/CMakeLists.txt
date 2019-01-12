##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.11)
project(supervisor)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(formation_control REQUIRED)
find_package(pycopter REQUIRED)
include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${formation_control_INCLUDE_DIRS}
  ${pycopter_INCLUDE_DIRS}
)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES supervisor
  CATKIN_DEPENDS roscpp rospy std_msgs 
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here


##############################################################################
# Sections
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5 COMPONENTS Quick QuickWidgets REQUIRED)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/View/*.ui)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/supervisor/View/*.h)

QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/View/*.cpp)

##############################################################################
# Binaries
##############################################################################
add_executable(View ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_executable(supervisor_main src/Model/supervisor.cpp)

add_dependencies(supervisor_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(supervisor_main ${catkin_LIBRARIES})
target_link_libraries(View Qt5::Widgets Qt5::Quick Qt5::QuickWidgets ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS View RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add modules for qt the easy way
#qt5_use_modules(lora_ground_control positioning)

