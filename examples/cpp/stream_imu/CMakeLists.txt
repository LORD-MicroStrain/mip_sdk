cmake_minimum_required(VERSION 3.16)

project("stream_imu_example_cpp"
    VERSION "1.0.0"
    DESCRIPTION "Simple example for streaming IMU data using C++"
    LANGUAGES C
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED)

# TODO: Set MIP_SDK_ROOT_DIR if building as a standalone project
if(DEFINED MIP_SDK_ROOT_DIR OR DEFINED CACHE{MIP_SDK_ROOT_DIR})
    set(_MIP_SDK_ROOT_DIR "${MIP_SDK_ROOT_DIR}")
elseif(DEFINED ENV{MIP_SDK_ROOT_DIR})
    set(_MIP_SDK_ROOT_DIR "$ENV{MIP_SDK_ROOT_DIR}")
else()
    set(_MIP_SDK_ROOT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
endif()

# We need to find MicroStrain and MIP SDK in a standalone project
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_LIST_DIR})
    list(APPEND CMAKE_PREFIX_PATH "${_MIP_SDK_ROOT_DIR}")

    find_package(mip REQUIRED CONFIG)

    # This example uses a serial connection and requires the microstrain_serial component
    find_package(microstrain REQUIRED COMPONENTS microstrain_serial CONFIG)
endif()

# Make sure the MIP libraries are found
if(NOT DEFINED CACHE{MIP_LIBRARIES} AND NOT DEFINED MIP_LIBRARIES)
    message(FATAL_ERROR "Could not find the required MIP libraries.")
endif()

# Make sure the MicroStrain libraries are found
if(NOT DEFINED CACHE{MICROSTRAIN_LIBRARIES} AND NOT DEFINED MICROSTRAIN_LIBRARIES)
    message(FATAL_ERROR "Could not find the required MicroStrain libraries.")
endif()

# Create the example
add_executable(${PROJECT_NAME}
    "${CMAKE_CURRENT_LIST_DIR}/stream_imu_example.cpp"
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${MIP_LIBRARIES}
    ${MICROSTRAIN_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${MIP_INCLUDE_DIRS}
    ${MICROSTRAIN_INCLUDE_DIRS}
)
