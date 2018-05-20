cmake_minimum_required(VERSION 2.8.3)
project(draw_cube)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs message_generation)

## Add in message and service files
add_message_files(FILES raven_automove.msg raven_state.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Include directories
include_directories(include ${catkin_INCLUDE_DIRS})
