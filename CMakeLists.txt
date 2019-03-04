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
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools
  roscpp
  rospy
  sensor_msgs
  tf
  tf_conversions
  trajectory_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

add_executable(leg_move_interface_test scripts/leg_move_interface_test.cpp)
target_link_libraries(leg_move_interface_test ${catkin_LIBRARIES})


add_executable(add_object scripts/add_object.cpp)
target_link_libraries(add_object ${catkin_LIBRARIES})


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})