cmake_minimum_required(VERSION 3.16)

project("7_series_gnss_ins_example_cpp"
    VERSION "1.0.0"
    DESCRIPTION "Simple example for the 3DM-GQ7-GNSS/INS, and 3DM-CV7-GNSS/INS using C++"
    LANGUAGES C CXX
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
elseif(NOT MICROSTRAIN_ENABLE_SERIAL)
    # microstrain_serial library is needed for source built projects
    message(FATAL_ERROR "The serial connections library is required for this example")
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
    "${CMAKE_CURRENT_LIST_DIR}/7_series_gnss_ins_example.cpp"
)

# Configure the project in a sub-folder for IDEs that support it
if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_LIST_DIR})
    file(RELATIVE_PATH PROJECT_RELATIVE_PATH "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_LIST_DIR}/..")
    set_target_properties(${PROJECT_NAME} PROPERTIES
        FOLDER "${PROJECT_RELATIVE_PATH}"
    )
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${MIP_LIBRARIES}
    ${MICROSTRAIN_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${MIP_INCLUDE_DIRS}
    ${MICROSTRAIN_INCLUDE_DIRS}
)

if(MSVC)
    set(COMPILE_OPTIONS
        "/W4"             # Enable warnings
        "/WX"             # Warnings as errors
        "/Zc:__cplusplus" # Enable updated __cplusplus value
        "/MP"             # Multi-process compilation
    )
else()
    set(COMPILE_OPTIONS
        "-Wall"   # Enable warnings
        "-Wextra" # Enable extra warnings
        "-Werror" # Warnings as errors
    )
endif()

target_compile_options(${PROJECT_NAME} PRIVATE
    "${COMPILE_OPTIONS}"
)
