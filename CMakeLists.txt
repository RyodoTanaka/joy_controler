cmake_minimum_required(VERSION 2.8.3)
project(joy_twist)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_twist_node src/joy_twist_node.cpp)
target_link_libraries(joy_twist_node
  ${catkin_LIBRARIES}
)
