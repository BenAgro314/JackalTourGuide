cmake_minimum_required(VERSION 2.8.3)
project(jackal_velodyne)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  tf
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  rosbag
  actionlib
  move_base_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
)

find_package(gazebo REQUIRED)

add_message_files(
    FILES
	tuple.msg
#   Message2.msg
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs rosbag actionlib move_base_msgs geometry_msgs message_runtime 
  INCLUDE_DIRS include
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
)



add_executable(tf_publisher src/tf_publisher.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})

add_executable(get_set_robot_position src/get_set_robot_position.cpp)
target_link_libraries(get_set_robot_position ${catkin_LIBRARIES})

add_executable(points_and_pose src/points_and_pose.cpp)
add_dependencies(points_and_pose points_and_pose_generate_messages_cpp)
target_link_libraries(points_and_pose ${catkin_LIBRARIES})

add_executable(navigation_goals src/navigation_goals.cpp)
add_dependencies(navigation_goals navigation_goals_messages_cpp)
target_link_libraries(navigation_goals ${catkin_LIBRARIES})


add_executable(tour_maker src/tour_maker.cpp)
add_dependencies(tour_maker tour_maker_messages_cpp)
target_link_libraries(tour_maker ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(diagnostics src/diagnostics.cpp)
add_dependencies(diagnostics diagnostics_cpp)
target_link_libraries(diagnostics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})