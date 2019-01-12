cmake_minimum_required(VERSION 2.8.3)
project(supervisor)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  #qt_build
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(formation_control REQUIRED)
find_package(pycopter REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES supervisor
  CATKIN_DEPENDS roscpp rospy std_msgs  
 # DEPENDS   

)

###########
## Build ##
###########

include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${formation_control_INCLUDE_DIRS}
)

#add_executable(view src/View/main.cpp)

add_executable(supervisor_main src/Model/supervisor.cpp)
add_dependencies(supervisor_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(supervisor_main ${catkin_LIBRARIES})

