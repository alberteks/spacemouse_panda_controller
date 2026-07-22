cmake_minimum_required(VERSION 3.0.2)
project(panda_arm_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(panda_controller_node src/joint_impedance_ik_controller.cpp)

target_link_libraries(panda_controller_node
  ${catkin_LIBRARIES}
)
