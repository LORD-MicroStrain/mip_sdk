
cmake_minimum_required(VERSION 3.10)


project(mscl_embedded C)


set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")

#
# MIP
#

set(MIP_DIR "${SRC_DIR}/mip")

set(MIP_SOURCES
    "${MIP_DIR}/mip_packet.c"
    "${MIP_DIR}/mip_packet.h"
)

include_directories(
    "${SRC_DIR}"
)

add_library(mip ${MIP_SOURCES})


#
# TESTING
#

include(CTest)

if(BUILD_TESTING)
    add_subdirectory("test")
endif()
