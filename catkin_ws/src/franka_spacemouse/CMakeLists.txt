cmake_minimum_required(VERSION 3.15)
project(franka_spacemouse)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${HIDAPI_INCLUDE_DIRS}
)

add_executable(spacemouse_node
  src/spacemouse_node.cpp
)

target_include_directories(spacemouse_node PRIVATE ${HIDAPI_INCLUDE_DIRS})

target_link_directories(spacemouse_node PRIVATE ${HIDAPI_LIBRARY_DIRS})

target_link_libraries(spacemouse_node
  PRIVATE
    ${catkin_LIBRARIES}
    ${HIDAPI_LIBRARIES}
)