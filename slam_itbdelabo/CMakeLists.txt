cmake_minimum_required(VERSION 3.0.2)
project(slam_itbdelabo)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  geometry_msgs
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
# INCLUDE_DIRS include
  LIBRARIES irbot
  CATKIN_DEPENDS nav_msgs roscpp rospy geometry_msgs tf std_msgs message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/irbot.cpp
# )

## Declare a C++ executable
# tf_transform_publisher
add_executable(tf_transform_publisher_itbdelabo src/tf_transform_publisher.cpp)
target_link_libraries (tf_transform_publisher_itbdelabo ${catkin_LIBRARIES})
add_dependencies(tf_transform_publisher_itbdelabo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
