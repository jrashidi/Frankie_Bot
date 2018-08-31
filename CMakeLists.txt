cmake_minimum_required(VERSION 2.8.3)
project(frankie_bot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES linorobot
  CATKIN_DEPENDS roscpp rospy tf nav_msgs std_msgs geometry_msgs sensor_msgs std_msgs
  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)