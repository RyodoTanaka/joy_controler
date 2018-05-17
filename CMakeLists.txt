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

# Install
install(TARGETS joy_twist_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
