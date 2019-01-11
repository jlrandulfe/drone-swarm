cmake_minimum_required(VERSION 2.8.3)
project(formation_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)



## Generate services in the 'srv' folder
  add_service_files(
    FILES
    Formation.srv
  )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES formation_control
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime 
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(execute src/frame.cpp)
add_dependencies(execute ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(execute ${catkin_LIBRARIES})

add_executable(srv_call src/srv_call.cpp)
add_dependencies(srv_call ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(srv_call ${catkin_LIBRARIES})

add_library(formation_control src/frame.cpp)
target_link_libraries(formation_control ${catkin_LIBRARIES})


 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
