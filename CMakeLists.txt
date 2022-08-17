
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(
    libmip
    VERSION "0.0.01"
    DESCRIPTION "MicroStrain Communications Library for embedded systems"
    LANGUAGES C CXX
)

set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")
set(EXT_DIR "${CMAKE_CURRENT_LIST_DIR}/ext")

include_directories(
    "${SRC_DIR}"
)

add_subdirectory("${EXT_DIR}")

if(WITH_INTERNAL)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/int")
endif()

set(MIP_DIR "${SRC_DIR}/mip")


#
# Build options
#

option(MSCL_TIMESTAMP_TYPE, "Override the type used for received data timestamps and timeouts (must be unsigned or at least 64 bits).")


#
# Utils
#

set(UTILS_DIR "${MIP_DIR}/utils")

set(UTILS_SOURCES
    "${UTILS_DIR}/byte_ring.c"
    "${UTILS_DIR}/byte_ring.h"
    "${UTILS_DIR}/serialization.c"
    "${UTILS_DIR}/serialization.h"
    "${UTILS_DIR}/enum_wrapper.hpp"
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

set(ALL_MIP_SOURCES ${MIPDEF_SOURCES} ${MIPDEF_CPP_SOURCES} ${MIPDEV_SOURCES} ${MIP_SOURCES} ${UTILS_SOURCES} ${MIP_CPP_HEADERS})


option(MSCL_DISABLE_CPP "Excludes all C++ files from the project." OFF)
if(MSCL_DISABLE_CPP)
    list(FILTER ALL_MIP_SOURCES EXCLUDE REGEX "[c|h]pp$")
endif()


add_library(mip ${ALL_MIP_SOURCES})


if(${MSCL_TIMESTAMP_TYPE})
    add_compile_definitions("MSCL_TIMESTAMP_TYPE=${MSCL_TIMESTAMP_TYPE}")
endif()

if(MSVC)
  target_compile_options(mip PRIVATE /W4)
else()
  target_compile_options(mip PRIVATE -Wall -Wextra)
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

option(BUILD_EXAMPLES "Builds the example programs." OFF)

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

    if(BUILD_DOCUMENTATION_FULL)
        set(DOXYGEN_INTERNAL_DOCS YES)
        set(DOXYGEN_WARN_AS_ERROR YES)
    else()
        set(DOXYGEN_HIDE_UNDOC_MEMBERS YES)
        set(DOXYGEN_HIDE_UNDOC_CLASSES YES)
    endif()

    if(BUILD_DOCUMENTATION_QUIET)
        set(DOXYGEN_QUIET YES)
    endif()

    doxygen_add_docs(docs
        "${SRC_DIR}"
        COMMENT "Generating documentation."
    )

endif(BUILD_DOCUMENTATION)

#
# Packaging
#

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CONFIG_EXPORT_DIR "${CMAKE_INSTALL_DATADIR}/cmake/mip")

set(EXPORT_TARGETS mip)
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

#
# Installation
#

install(DIRECTORY "${SRC_DIR}/mip" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" FILES_MATCHING PATTERN "*.h*")
install(TARGETS mip EXPORT mip-targets ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}/")

install(EXPORT
    mip-targets
    DESTINATION "${CONFIG_EXPORT_DIR}"
)

install(FILES
    "${CMAKE_BINARY_DIR}/mip-config.cmake"
    "${CMAKE_BINARY_DIR}/mip-config-version.cmake"
    DESTINATION "${CONFIG_EXPORT_DIR}"
)

