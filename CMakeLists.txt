
cmake_minimum_required(VERSION 3.10)


project(
    MSCL-Embedded
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

#
# Utils
#

set(UTILS_DIR "${SRC_DIR}/utils")

set(UTILS_SOURCES
    "${UTILS_DIR}/byte_ring.c"
    "${UTILS_DIR}/byte_ring.h"
    "${UTILS_DIR}/serialization.c"
    "${UTILS_DIR}/serialization.h"
)

#
# MIP Control
#

set(MIP_DIR "${SRC_DIR}/mip")

set(MIP_SOURCES
    "${MIP_DIR}/mip_packet.c"
    "${MIP_DIR}/mip_packet.h"
    "${MIP_DIR}/mip_parser.c"
    "${MIP_DIR}/mip_parser.h"
    "${MIP_DIR}/mip_field.c"
    "${MIP_DIR}/mip_field.h"
    "${MIP_DIR}/definitions/descriptors.c"
    "${MIP_DIR}/definitions/descriptors.h"
    "${MIP_DIR}/mip.hpp"
    "${MIP_DIR}/mip_cmdqueue.c"
    "${MIP_DIR}/mip_cmdqueue.h"
    "${MIP_DIR}/mip_device.c"
    "${MIP_DIR}/mip_device.h"
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
)

add_library(mip ${MIPDEF_SOURCES} ${MIP_SOURCES} ${UTILS_SOURCES} ${MIP_CPP_HEADERS})


#
# TESTING
#

include(CTest)

# CTest adds a BUILD_TESTING option.
if(BUILD_TESTING)
    add_subdirectory("test")
endif()

#
# EXAMPLES
#

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

    doxygen_add_docs(mscl_embedded
        "${SRC_DIR}"
        COMMENT "Generating documentation."
    )

endif(BUILD_DOCUMENTATION)
