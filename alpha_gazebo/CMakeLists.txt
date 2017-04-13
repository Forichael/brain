cmake_minimum_required(VERSION 2.8.3)
project(alpha_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	roscpp
	rospy
	sensor_msgs
	std_msgs
	gazebo_ros
	)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBARRY_DIRS})

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${GAZEBO_INCLUDE_DIRS}
	include/
	)

catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES alpha_action
	CATKIN_DEPENDS actionlib_msgs
	DEPENDS roscpp gazebo_ros
	#  DEPENDS system_lib
	)

## Declare a C++ library
add_library(alpha_fake_gripper SHARED
  src/fake_gripper.cpp
)
target_link_libraries(alpha_fake_gripper
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
	)


