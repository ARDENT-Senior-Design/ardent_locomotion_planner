cmake_minimum_required(VERSION 2.8.3)
project(ardent_locomotion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_filters
  roscpp
  sensor_msgs
  tf
  tf_conversions
  trajectory_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
