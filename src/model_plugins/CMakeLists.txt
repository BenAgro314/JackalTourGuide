cmake_minimum_required(VERSION 3.0.2)
project(model_plugins)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)


find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(vehical_plugin src/vehical_plugin.cpp)
target_link_libraries(vehical_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
	DEPENDS gazebo_ros roscpp rospy std_msgs
)


