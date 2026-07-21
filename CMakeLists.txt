cmake_minimum_required(VERSION 3.15)
project(SpacemouseArmController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)
find_package(Franka REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(spacemouse_read_test
    src/main.cpp
    src/spacemouse.cpp
    src/spacemouse_test_only.cpp
    src/shared_state.cpp
    src/examples_common.cpp
)

target_include_directories(spacemouse_read_test PRIVATE include ${HIDAPI_INCLUDE_DIRS})

target_link_libraries(spacemouse_read_test
    PRIVATE 
        Threads::Threads
        Franka::Franka
        ${HIDAPI_LIBRARIES}
        Eigen3::Eigen
)

target_link_directories(spacemouse_read_test PRIVATE ${HIDAPI_LIBRARY_DIRS})

add_executable(control_print_test
    src/control_read_main.cpp
    src/spacemouse.cpp
    src/test_spacemouse_reading.cpp
    src/shared_state.cpp
    src/examples_common.cpp
)

target_include_directories(control_print_test PRIVATE include ${HIDAPI_INCLUDE_DIRS})

target_link_libraries(control_print_test
    PRIVATE 
        Threads::Threads
        Franka::Franka
        ${HIDAPI_LIBRARIES}
        Eigen3::Eigen
)

target_link_directories(control_print_test PRIVATE ${HIDAPI_LIBRARY_DIRS})

add_executable(control_test
    src/control_main.cpp
    src/spacemouse.cpp
    src/arm_ctrl.cpp
    src/shared_state.cpp
    src/examples_common.cpp
)

target_include_directories(control_test PRIVATE include ${HIDAPI_INCLUDE_DIRS})

target_link_libraries(control_test
    PRIVATE 
        Threads::Threads
        Franka::Franka
        ${HIDAPI_LIBRARIES}
        Eigen3::Eigen
)

target_link_directories(control_test PRIVATE ${HIDAPI_LIBRARY_DIRS})
