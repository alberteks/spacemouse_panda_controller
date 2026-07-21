cmake_minimum_required(VERSION 3.15)
project(SpacemouseArmController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(Franka REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(spacemouse_teleop_test
    src/main.cpp
    src/spacemouse.cpp
    src/test_spacemouse_reading.cpp
    src/shared_state.cpp
    src/examples_common.cpp
)

target_include_directories(spacemouse_teleop_test PRIVATE include ${HIDAPI_INCLUDE_DIRS})

target_link_libraries(spacemouse_teleop_test 
    PRIVATE 
        Threads::Threads
        Franka::Franka
        ${HIDAPI_LIBRARIES}
        Eigen3::Eigen
)

target_link_directories(spacemouse_teleop_test PRIVATE ${HIDAPI_LIBRARY_DIRS})

set_target_properties(spacemouse_teleop_test PROPERTIES 
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../bin"
)