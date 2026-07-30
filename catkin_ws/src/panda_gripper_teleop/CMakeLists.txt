cmake_minimum_required(VERSION 3.0.2)
project(panda_gripper_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  franka_gripper
  actionlib
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    franka_gripper
    actionlib
    roscpp
    sensor_msgs
    std_msgs
)

# Include header paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Build executable for gripper teleop node
add_library(${PROJECT_NAME}
  src/gripper_teleop_node.cpp
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)