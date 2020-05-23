cmake_minimum_required(VERSION 3.0.2)
project(custom_plugins)

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

add_library(actor_plugin src/actor_plugin.cpp)
target_link_libraries(actor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(randomwalk_plugin src/randomwalk_plugin.cpp)
target_link_libraries(randomwalk_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(boids_plugin src/boids_plugin.cpp)
target_link_libraries(boids_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(random_place_plugin src/random_place_plugin.cpp)
target_link_libraries(random_place_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(TrajectoryActorPlugin src/TrajectoryActorPlugin.cc)
target_link_libraries(TrajectoryActorPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
	DEPENDS gazebo_ros roscpp rospy std_msgs
)


