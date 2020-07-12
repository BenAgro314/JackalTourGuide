cmake_minimum_required(VERSION 3.0.2)
project(dashboard)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  ros_numpy
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES dashboard
   CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs nav_msgs pcl_conversions pcl_ros ros_numpy rosbag roscpp rospy sensor_msgs std_msgs tf2_msgs
   DEPENDS system_lib
)

catkin_install_python(PROGRAMS
        src/math_utilities.py
        src/plot_utilities.py
       	src/data_processing.py
        src/bag_tools.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

