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


catkin_package(
	DEPENDS gazebo_ros roscpp rospy std_msgs
	INCLUDE_DIRS src/include
)



link_directories(src/include ${GAZEBO_LIBRARY_DIRS})
include_directories(src/include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(Perlin src/include/Perlin.cpp)
target_link_libraries(Perlin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(utilities src/include/utilities.cpp)
target_link_libraries(utilities ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(sdfstring src/include/sdfstring.cpp)
target_link_libraries(sdfstring ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(vehicles src/vehicles.cpp)
target_link_libraries(vehicles utilities Perlin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(world_entities src/world_entities.cpp)
target_link_libraries(world_entities sdfstring utilities ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(quadtree src/include/quadtree.cpp)
target_link_libraries(quadtree utilities ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(puppeteer src/puppeteer.cpp)
target_link_libraries(puppeteer utilities quadtree vehicles ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(world_factory src/world_factory.cpp) 
target_link_libraries(world_factory world_entities ${catkin_LIBRARIES})

add_executable(test_file src/tests.cpp) 
target_link_libraries(test_file world_entities quadtree ${catkin_LIBRARIES})

