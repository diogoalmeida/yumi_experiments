cmake_minimum_required(VERSION 2.8.3)
project(yumi_experiments)

find_package(
  catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  folding_assembly_controller
  yumi_interface
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS rospy actionlib actionlib_msgs folding_assembly_controller yumi_interface
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
