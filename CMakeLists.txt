cmake_minimum_required(VERSION 3.16)
project(cerebellum LANGUAGES C CXX)

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

# Third-party: RobStride motor driver (vendored verbatim, do not modify)
add_library(robstride_driver STATIC
    third_party/robstride/src/motor_cfg.cpp
)
target_include_directories(robstride_driver PUBLIC
    ${CMAKE_SOURCE_DIR}/third_party/robstride/include
)
target_compile_features(robstride_driver PUBLIC cxx_std_17)

# Third-party: HiPNUC IMU decoder core + Linux serial layer (vendored verbatim)
add_library(hipnuc STATIC
    third_party/hipnuc/hipnuc/hipnuc_data.c
    third_party/hipnuc/hipnuc/hipnuc_decoder.c
    third_party/hipnuc/hipnuc/hipnuc_nmea.c
    third_party/hipnuc/hipnuc/hipnuc_can.c
    third_party/hipnuc/hipnuc/hipnuc_i2c.c
)
target_include_directories(hipnuc PUBLIC ${CMAKE_SOURCE_DIR}/third_party/hipnuc/hipnuc)
set_target_properties(hipnuc PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES C_EXTENSIONS OFF)

add_library(hipnuc_serial STATIC
    third_party/hipnuc/serial/hipnuc_serial.c
    third_party/hipnuc/serial/hipnuc_serial_io.c
)
target_include_directories(hipnuc_serial PUBLIC ${CMAKE_SOURCE_DIR}/third_party/hipnuc/serial)
target_link_libraries(hipnuc_serial PUBLIC hipnuc)
set_target_properties(hipnuc_serial PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)

# Third-party: BMS Modbus RTU driver (vendored verbatim, POSIX serial backend)
if(UNIX)
    add_library(battery_bms STATIC
        third_party/battery/src/driver.cpp
        third_party/battery/src/serial_posix.cpp
    )
    target_include_directories(battery_bms PUBLIC ${CMAKE_SOURCE_DIR}/third_party/battery/include)
    target_compile_features(battery_bms PUBLIC cxx_std_17)
endif()

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
    src/drivers/robstride_motor_adapter.cpp
    src/drivers/power_board.cpp
    src/drivers/imu_driver.cpp
    src/drivers/hipnuc_imu_driver.cpp
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
    robstride_driver
    hipnuc_serial
)

# Nodes
set(NODES
    power_board_node
    battery_node
    command_node
    sensor_node
    rl_policy_node
    motion_control_node
    motor_driver_node
    safety_monitor_node
    logger_node
)

if(NOT UNIX)
    list(REMOVE_ITEM NODES battery_node)
endif()

foreach(node ${NODES})
    add_executable(${node} src/nodes/${node}.cpp)
    target_link_libraries(${node} PRIVATE cerebellum_core)
endforeach()

if(UNIX)
    target_link_libraries(battery_node PRIVATE battery_bms)
endif()

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
