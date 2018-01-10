cmake_minimum_required(VERSION 2.8.3)
project(yumi_experiments)

find_package(
  catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  yumi_interface
  generic_control_toolbox
  folding_assembly_controller
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

add_action_files(
	DIRECTORY action
	FILES RunFolding.action ApproachController.action
)

generate_messages(
  DEPENDENCIES
	actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy actionlib actionlib_msgs folding_assembly_controller yumi_interface generic_control_toolbox
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(approach_action_node src/approach_action_node.cpp src/approach_controller.cpp)
target_link_libraries(approach_action_node ${catkin_LIBRARIES})
add_dependencies(approach_action_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
