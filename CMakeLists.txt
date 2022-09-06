cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(LIBRARY_NAME mip)

# Used for conditional cmake options
include(CMakeDependentOption)

# Need to set this before the project for usage of CMAKE_TOOLCHAIN_FILE
option(BUILD_ARDUINO "Build the library for Arduino support" OFF)

if(BUILD_ARDUINO)
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arduino/Arduino-Toolchain.cmake"
        CACHE
        FILEPATH
        "Toolchain file used for Arduino builds."
        FORCE)
endif()

project(
    "MIP SDK"
    VERSION "0.0.01"
    DESCRIPTION "MicroStrain Communications Library for embedded systems"
    HOMEPAGE_URL "https://github.com/LORD-MicroStrain/libmip"
    LANGUAGES C CXX
)

set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")
set(EXT_DIR "${CMAKE_CURRENT_LIST_DIR}/ext")

include_directories(
    "${SRC_DIR}"
)

if(WITH_INTERNAL)
    set(MIP_INTERNAL_DIR "int" CACHE PATH "")
    if(MIP_INTERNAL_DIR)
        add_subdirectory("${MIP_INTERNAL_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/mip-internal")
    endif()
endif()

set(MIP_DIR "${SRC_DIR}/mip")


#
# Build options
#

# Only allow this option on Arduino builds. Will be ON with Arduino builds and OFF by default for normal builds
cmake_dependent_option(AUTO_SET_SKETCHBOOK_PATH "Find the Arduino sketchbook path." OFF "BUILD_ARDUINO" ON)

# Only allow these options not on Arduino builds. Will be OFF with Arduino builds and ON by default for normal builds
cmake_dependent_option(WITH_SERIAL "Build serial connection support into the library and examples" ON "NOT BUILD_ARDUINO" OFF)
cmake_dependent_option(WITH_TCP    "Build TCP connection support into the library and examples"    ON "NOT BUILD_ARDUINO" OFF)

set(MIP_TIMESTAMP_TYPE "" CACHE STRING "Override the type used for received data timestamps and timeouts (must be unsigned or at least 64 bits).")


#
# Utils
#

set(UTILS_DIR "${MIP_DIR}/utils")

set(UTILS_SOURCES
    "${UTILS_DIR}/byte_ring.c"
    "${UTILS_DIR}/byte_ring.h"
    "${UTILS_DIR}/serialization.c"
    "${UTILS_DIR}/serialization.h"
)

#
# MIP Control
#

set(MIP_SOURCES
    "${MIP_DIR}/mip_cmdqueue.c"
    "${MIP_DIR}/mip_cmdqueue.h"
    "${MIP_DIR}/mip_dispatch.c"
    "${MIP_DIR}/mip_dispatch.h"
    "${MIP_DIR}/mip_field.c"
    "${MIP_DIR}/mip_field.h"
    "${MIP_DIR}/mip_offsets.h"
    "${MIP_DIR}/mip_packet.c"
    "${MIP_DIR}/mip_packet.h"
    "${MIP_DIR}/mip_parser.c"
    "${MIP_DIR}/mip_parser.h"
    "${MIP_DIR}/mip_result.c"
    "${MIP_DIR}/mip_result.h"
    "${MIP_DIR}/mip_types.h"
    "${MIP_DIR}/definitions/descriptors.c"
    "${MIP_DIR}/definitions/descriptors.h"
    "${MIP_DIR}/mip.hpp"
    "${MIP_DIR}/mip_all.h"
)

set(MIPDEV_SOURCES
    "${MIP_DIR}/mip_interface.c"
    "${MIP_DIR}/mip_interface.h"
    "${MIP_DIR}/mip_device.cpp"
    "${MIP_DIR}/mip_device.hpp"
)

set(MIPDEF_SOURCES
    "${MIP_DIR}/definitions/commands_3dm.c"
    "${MIP_DIR}/definitions/commands_3dm.h"
    "${MIP_DIR}/definitions/commands_base.c"
    "${MIP_DIR}/definitions/commands_base.h"
    "${MIP_DIR}/definitions/commands_filter.c"
    "${MIP_DIR}/definitions/commands_filter.h"
    "${MIP_DIR}/definitions/commands_gnss.c"
    "${MIP_DIR}/definitions/commands_gnss.h"
    "${MIP_DIR}/definitions/commands_rtk.c"
    "${MIP_DIR}/definitions/commands_rtk.h"
    "${MIP_DIR}/definitions/commands_system.c"
    "${MIP_DIR}/definitions/commands_system.h"
    "${MIP_DIR}/definitions/data_filter.c"
    "${MIP_DIR}/definitions/data_filter.h"
    "${MIP_DIR}/definitions/data_gnss.c"
    "${MIP_DIR}/definitions/data_gnss.h"
    "${MIP_DIR}/definitions/data_sensor.c"
    "${MIP_DIR}/definitions/data_sensor.h"
    "${MIP_DIR}/definitions/data_shared.c"
    "${MIP_DIR}/definitions/data_shared.h"
    "${MIP_DIR}/definitions/data_system.c"
    "${MIP_DIR}/definitions/data_system.h"
    ${INTDEF_SOURCES}
)

string(REPLACE ".h" ".hpp" MIPDEF_HPP_SOURCES "${MIPDEF_SOURCES}")
string(REPLACE ".c" ".cpp" MIPDEF_CPP_SOURCES "${MIPDEF_HPP_SOURCES}")

if(WITH_SERIAL)
    list(APPEND UTILS_SOURCES
        "${UTILS_DIR}/serial_port.c"
        "${UTILS_DIR}/serial_port.h"
    )
    list(APPEND MIP_INTERFACE_SOURCES
        "${MIP_DIR}/platform/serial_connection.hpp"
        "${MIP_DIR}/platform/serial_connection.cpp"
    )
endif()
if(WITH_TCP)
    list(APPEND UTILS_SOURCES
        "${UTILS_DIR}/tcp_socket.c"
        "${UTILS_DIR}/tcp_socket.h"
    )
    list(APPEND MIP_INTERFACE_SOURCES
        "${MIP_DIR}/platform/tcp_connection.hpp"
        "${MIP_DIR}/platform/tcp_connection.cpp"
    )
endif()

set(ALL_MIP_SOURCES
    ${MIPDEF_SOURCES}
    ${MIPDEF_CPP_SOURCES}
    ${MIPDEV_SOURCES}
    ${MIP_SOURCES}
    ${UTILS_SOURCES}
    ${MIP_CPP_HEADERS}
    ${MIP_INTERFACE_SOURCES}
)

# Only allow this option if not an Arduino build. Will be ON with Arduino builds and OFF by default for normal builds
cmake_dependent_option(MIP_DISABLE_CPP "Excludes all C++ files from the project." OFF "NOT BUILD_ARDUINO" ON)
if(MIP_DISABLE_CPP)
    list(FILTER ALL_MIP_SOURCES EXCLUDE REGEX "[c|h]pp$")
endif()

if(BUILD_ARDUINO)
    get_board_id(board_id mega atmega2560)
    arduino_cmake_project(${LIBRARY_NAME} BOARD_NAME mega BOARD_CPU atmega2560)
#    _add_arduino_cmake_library(${LIBRARY_NAME}_AVR ${ALL_MIP_SOURCES})
#    add_arduino_header_only_library(${LIBRARY_NAME} ${board_id} ${ALL_MIP_SOURCES})
#    foreach(filename ${ALL_MIP_SOURCES})
#        message(STATUS "filename: ${filename}")
#    endforeach()
    add_arduino_library(${LIBRARY_NAME} "${ALL_MIP_SOURCES}")
else()
    add_library(${LIBRARY_NAME} ${ALL_MIP_SOURCES})
endif()

#add_library(${LIBRARY_NAME} ${ALL_MIP_SOURCES})

if(BUILD_ARDUINO)
#    get_board_id(board_id mega atmega2560)
#    arduino_cmake_project(${LIBRARY_NAME} BOARD_NAME mega BOARD_CPU atmega2560)
#    add_arduino_header_only_library()
#    add_arduino_executable(Arduino_${LIBRARY_NAME})
endif()


if(${MIP_TIMESTAMP_TYPE})
    add_compile_definitions("MIP_TIMESTAMP_TYPE=${MIP_TIMESTAMP_TYPE}")
endif()

if(MSVC)
  target_compile_options(${LIBRARY_NAME} PRIVATE /W4)
else()
  target_compile_options(${LIBRARY_NAME} PRIVATE -Wall -Wextra)
endif()

#
# TESTING
#

# CTest defines this option to ON by default, so override it to OFF here.
option(BUILD_TESTING "Build the testing tree." OFF)

include(CTest)

if(BUILD_TESTING)
    add_subdirectory("test")
endif()

#
# EXAMPLES
#

option(BUILD_EXAMPLES "Builds the example programs." ON)

if(BUILD_EXAMPLES)
    add_subdirectory("examples")
endif()

#
# DOCUMENTATION
#

find_package(Doxygen)
option(BUILD_DOCUMENTATION       "Build the documentation." OFF)
option(BUILD_DOCUMENTATION_FULL  "Build the full (internal) documentation." OFF)
option(BUILD_DOCUMENTATION_QUIET "Suppress doxygen standard output." ON)

if(BUILD_DOCUMENTATION)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is required to build documentation.")
    endif()

    set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/documentation")
    set(DOXYGEN_IMAGE_PATH "${CMAKE_CURRENT_LIST_DIR}/docs")

    set(DOXYGEN_WARN_IF_UNDOCUMENTED NO)

    if(NOT MIP_DISABLE_CPP)
        set(DOXYGEN_PREDEFINED "__cplusplus")
    endif()

    set(DOXYGEN_EXTRACT_ALL YES)

    if(BUILD_DOCUMENTATION_FULL)
        set(DOXYGEN_INTERNAL_DOCS YES)
        # set(DOXYGEN_WARN_AS_ERROR YES)
    else()
        set(DOXYGEN_HIDE_UNDOC_MEMBERS YES)
        set(DOXYGEN_HIDE_UNDOC_CLASSES YES)
    endif()

    if(BUILD_DOCUMENTATION_QUIET)
        set(DOXYGEN_QUIET YES)
    endif()

    doxygen_add_docs(docs
        "${SRC_DIR}" "${CMAKE_CURRENT_LIST_DIR}/docs"
        COMMENT "Generating documentation."
    )

endif(BUILD_DOCUMENTATION)

#
# Packaging
#

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CONFIG_EXPORT_DIR "${CMAKE_INSTALL_DATADIR}/cmake/mip")

set(EXPORT_TARGETS ${LIBRARY_NAME})
configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/mip-config.cmake.in"
    "${CMAKE_BINARY_DIR}/mip-config.cmake"
    INSTALL_DESTINATION "${CONFIG_EXPORT_DIR}"
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR CONFIG_EXPORT_DIR
)
unset(EXPORT_TARGETS)

write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/mip-config-version.cmake"
    COMPATIBILITY AnyNewerVersion
)

if(BUILD_ARDUINO)
    # Make the Output directory which is where we will copy all of the build artifacts on a successful build
    set(ARDUINO_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ArduinoLibrary)

    # Copy the artifacts to an Output directory to make it easier for users to navigate the artifacts
    add_custom_command(TARGET ${LIBRARY_NAME} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E rm -Rf ${ARDUINO_OUTPUT_DIR}
                       COMMAND ${CMAKE_COMMAND} -E make_directory ${ARDUINO_OUTPUT_DIR}/src/atmega2560
                       COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}> ${ARDUINO_OUTPUT_DIR}/src/atmega2560
                       COMMAND ${CMAKE_COMMAND} -DOUTPUT_DIR=${ARDUINO_OUTPUT_DIR}
                                                -DLIBRARY_NAME=${LIBRARY_NAME}
                                                -DVERSION=${CMAKE_PROJECT_VERSION}
                                                -DDESCRIPTION=${CMAKE_PROJECT_DESCRIPTION}
                                                -DURL=${CMAKE_PROJECT_HOMEPAGE_URL}
                                                -DSOURCE_DIR=${MIP_DIR}/
                                                -P ${CMAKE_SOURCE_DIR}/cmake/arduino/GenerateArduinoLibrary.cmake
                       )
endif()

#
# Installation
#

if(NOT BUILD_ARDUINO)
    install(DIRECTORY "${SRC_DIR}/mip" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" FILES_MATCHING PATTERN "*.h*")
    install(TARGETS ${LIBRARY_NAME} EXPORT mip-targets ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}/")

    install(EXPORT
        mip-targets
        DESTINATION "${CONFIG_EXPORT_DIR}"
    )

    install(FILES
        "${CMAKE_BINARY_DIR}/mip-config.cmake"
        "${CMAKE_BINARY_DIR}/mip-config-version.cmake"
        DESTINATION "${CONFIG_EXPORT_DIR}"
    )
endif()
