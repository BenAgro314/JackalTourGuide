cmake_minimum_required(VERSION 3.0.2)
project(world_generator)

add_definitions(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  pcl_ros
  roscpp
  rospy
  std_msgs
  pcl_conversions
  sensor_msgs
  rosbag
  tf2_msgs
)

find_package(gazebo REQUIRED)

link_directories( include/world_generator ${GAZEBO_LIBRARY_DIRS})
include_directories( include/world_generator ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
	DEPENDS gazebo_ros pcl_ros roscpp rospy std_msgs rosbag sensor_msgs pcl_conversions tf2_msgs
	INCLUDE_DIRS include/world_generator
)

add_library(objects src/objects.cpp ${catkin_INCLUDE_DIRS})
target_link_libraries(objects ${catkin_INCLUDE_DIRS} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(maze src/maze.cpp ${catkin_INCLUDE_DIRS})
target_link_libraries(maze objects ${catkin_INCLUDE_DIRS} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(world_master src/world_master.cpp ${catkin_INCLUDE_DIRS})
target_link_libraries(world_master objects maze ${catkin_INCLUDE_DIRS} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
