cmake_minimum_required(VERSION 3.0.2)
project(myhal_simulator)

add_definitions(-std=c++14)

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

add_library(utilities src/utilities.cpp)
target_link_libraries(utilities ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(vehicles src/vehicles.cpp)
target_link_libraries(vehicles utilities ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(vehicle_plugin src/vehicle_plugin.cpp)
target_link_libraries(vehicle_plugin vehicles ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
	DEPENDS gazebo_ros roscpp rospy std_msgs
)


