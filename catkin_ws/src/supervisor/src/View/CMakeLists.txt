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
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.

catkin_package()

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

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/supervisor/*.h)

QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
add_executable(View ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP})
	#build/ui_main_window.h)


target_link_libraries(View Qt5::Widgets Qt5::Quick Qt5::QuickWidgets ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS View RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add modules for qt the easy way
#qt5_use_modules(lora_ground_control positioning)

