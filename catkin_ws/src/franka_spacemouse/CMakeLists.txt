cmake_minimum_required(VERSION 3.0.2)
project(franka_spacemouse)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(spacemouse_node src/spacemouse_node.cpp)
target_link_libraries(spacemouse_node ${catkin_LIBRARIES})
target_link_libraries(spacemouse_node ${catkin_LIBRARIES} hidapi)