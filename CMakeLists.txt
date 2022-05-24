
cmake_minimum_required(VERSION 3.10)


project(mscl_embedded C)


set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")

include_directories(
    "${SRC_DIR}"
)


#
# Utils
#

set(UTILS_DIR "${SRC_DIR}/utils")

set(UTILS_SOURCES
    "${UTILS_DIR}/byte_ring.c"
    "${UTILS_DIR}/byte_ring.h"
)

#
# MIP
#

set(MIP_DIR "${SRC_DIR}/mip")

set(MIP_SOURCES
    "${MIP_DIR}/mip_packet.c"
    "${MIP_DIR}/mip_packet.h"
    "${MIP_DIR}/mip_parser.c"
    "${MIP_DIR}/mip_parser.h"
)

add_library(mip ${MIP_SOURCES} ${UTILS_SOURCES})


#
# TESTING
#

include(CTest)

if(BUILD_TESTS)
    add_subdirectory("test")
endif()

#
# DOCUMENTATION
#

find_package(Doxygen)
option(BUILD_DOCUMENTATION "Build the documentation." OFF)

if(BUILD_DOCUMENTATION)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is required to build documentation.")
    endif()

    add_custom_target(documentation ALL
        COMMAND ${DOXYGEN_EXECUTABLE} "Doxyfile"
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        COMMENT "Generating documentation."
        VERBATIM
    )

endif(BUILD_DOCUMENTATION)
