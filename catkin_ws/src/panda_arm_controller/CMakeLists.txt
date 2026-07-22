cmake_minimum_required(VERSION 3.0.2)
project(panda_arm_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  franka_hw
  franka_msgs
  roscpp
  geometry_msgs
  realtime_tools
  tf2
  urdf
  kdl_parser
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED) 

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    pluginlib
    franka_hw
    franka_msgs
    roscpp
    geometry_msgs
    realtime_tools
    tf2
    urdf
    kdl_parser
  DEPENDS
    Eigen3
    orocos_kdl
)

# Include header paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# Build the controller shared library
add_library(${panda_arm_controller}
  src/joint_impedance_ik_controller.cpp
)

# Link dependencies
target_link_libraries(${panda_arm_controller}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

add_executable(panda_controller_node src/joint_impedance_ik_controller.cpp)

target_link_libraries(panda_controller_node
  ${catkin_LIBRARIES}
  ${HIDAPI_LIBRARIES}
)

target_include_directories(panda_controller_node PRIVATE ${HIDAPI_INCLUDE_DIRS})

target_link_directories(panda_controller_node PRIVATE ${HIDAPI_LIBRARY_DIRS})
