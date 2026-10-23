cmake_minimum_required(VERSION 3.16)
project(cerebellum LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(BUILD_TESTS "Build tests" ON)

# Find dependencies
find_package(Threads REQUIRED)
find_package(LCM REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

# Generate LCM types
file(GLOB LCM_TYPES ${CMAKE_SOURCE_DIR}/lcm_types/quad/*.lcm)
lcm_wrap_types(
    CPP_HEADERS LCM_HEADERS
    ${LCM_TYPES}
)

lcm_add_library(cerebellum_lcm CPP ${LCM_HEADERS})
target_include_directories(cerebellum_lcm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}  # for generated LCM headers
)

# Core library
set(CORE_SOURCES
    src/core/periodic_task.cpp
    src/core/lowpass_filter.cpp
    src/core/robot_config.cpp
    src/core/state.cpp
    src/core/state_machine.cpp
    src/core/static_state.cpp
    src/core/gait_state.cpp
)

set(DRIVER_SOURCES
    src/drivers/can_bus.cpp
    src/drivers/joint_motor.cpp
)

set(RL_SOURCES
    src/rl/policy_model.cpp
)

set(UTIL_SOURCES
    src/utils/logger.cpp
    src/utils/config_parser.cpp
)

add_library(cerebellum_core STATIC
    ${CORE_SOURCES}
    ${DRIVER_SOURCES}
    ${RL_SOURCES}
    ${UTIL_SOURCES}
)

target_link_libraries(cerebellum_core PUBLIC
    Threads::Threads
    Eigen3::Eigen
    yaml-cpp
    cerebellum_lcm
    lcm
)

# Nodes
set(NODES
    command_node
    sensor_node
    rl_policy_node
    motion_control_node
    motor_driver_node
    safety_monitor_node
    logger_node
)

foreach(node ${NODES})
    add_executable(${node} src/nodes/${node}.cpp)
    target_link_libraries(${node} PRIVATE cerebellum_core)
endforeach()

# Tests
if(BUILD_TESTS)
    add_executable(test_lcm tests/test_lcm.cpp)
    target_link_libraries(test_lcm PRIVATE cerebellum_core)

    add_executable(test_can tests/test_can.cpp)
    target_link_libraries(test_can PRIVATE cerebellum_core)
endif()

# Install
install(TARGETS ${NODES}
    RUNTIME DESTINATION bin
)

install(DIRECTORY config scripts
    DESTINATION share/${PROJECT_NAME}
)
