

option(MIP_DISABLE_CPP        "Excludes all C++ files from the project."                             OFF)
option(MIP_ENABLE_DIAGNOSTICS "Enable various diagnostic counters in the mip library for debugging." OFF)

set(MIP_TIMESTAMP_TYPE "uint64_t" CACHE STRING "Override the type used for received data timestamps and timeouts (must be unsigned or at least 64 bits).")

#
# Utils
#

set(UTILS_DIR "${MIP_DIR}/utils")

set(UTILS_SOURCES
    "${UTILS_DIR}/byte_ring.c"
    "${UTILS_DIR}/byte_ring.h"
)

#
# MIP Control
#

set(MIP_SOURCES_C
    "${VERSION_OUT_FILE}"
    "${MIP_DIR}/mip_cmdqueue.c"
    "${MIP_DIR}/mip_cmdqueue.h"
    "${MIP_DIR}/mip_descriptors.c"
    "${MIP_DIR}/mip_descriptors.h"
    "${MIP_DIR}/mip_dispatch.c"
    "${MIP_DIR}/mip_dispatch.h"
    "${MIP_DIR}/mip_field.c"
    "${MIP_DIR}/mip_field.h"
    "${MIP_DIR}/mip_interface.c"
    "${MIP_DIR}/mip_interface.h"
    "${MIP_DIR}/mip_offsets.h"
    "${MIP_DIR}/mip_packet.c"
    "${MIP_DIR}/mip_packet.h"
    "${MIP_DIR}/mip_parser.c"
    "${MIP_DIR}/mip_parser.h"
    "${MIP_DIR}/mip_result.c"
    "${MIP_DIR}/mip_result.h"
    "${MIP_DIR}/mip_types.h"
    "${MIP_DIR}/definitions/common.c"
    "${MIP_DIR}/definitions/common.h"
    "${MIP_DIR}/mip_all.h"
)

set(MIP_SOURCES_CPP
    "mip.hpp"
    "mip_all.hpp"
    "mip_cmdqueue.hpp"
    "mip_descriptors.hpp"
    "mip_field.hpp"
    "mip_packet.hpp"
    "mip_parser.hpp"
    "mip_result.hpp"
    "mip_interface.cpp"
    "mip_interface.hpp"
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
    "${MIP_DIR}/definitions/commands_aiding.c"
    "${MIP_DIR}/definitions/commands_aiding.h"
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

string(REGEX REPLACE "\.h(;|$)" ".hpp\\1" MIPDEF_HPP_SOURCES "${MIPDEF_SOURCES}")
string(REGEX REPLACE "\.c(;|$)" ".cpp\\1" MIPDEF_CPP_SOURCES "${MIPDEF_HPP_SOURCES}")

set(ALL_MIP_SOURCES
    ${MIPDEF_SOURCES}
    ${MIPDEF_CPP_SOURCES}
    ${MIP_SOURCES_C}
    ${MIP_SOURCES_CPP}
    ${UTILS_SOURCES}
    ${MIP_CPP_HEADERS}
    ${MIP_INTERFACE_SOURCES}
    ${MIP_EXTRA_SOURCES}
)

if(MIP_DISABLE_CPP)
    list(FILTER ALL_MIP_SOURCES EXCLUDE REGEX "[c|h]pp$")
endif()

add_library(mip ${ALL_MIP_SOURCES})



if(${MIP_ENABLE_DIAGNOSTICS})
    list(APPEND MIP_DEFINES "MIP_ENABLE_DIAGNOSTICS")
endif()

if(${MIP_TIMESTAMP_TYPE})
    list(APPEND MIP_DEFINES "MIP_TIMESTAMP_TYPE=${MIP_TIMESTAMP_TYPE}")
endif()

target_compile_definitions(mip PUBLIC "${MIP_DEFINES}")

target_link_libraries(mip PUBLIC microstrain_common)

#target_include_directories(mip INTERFACE ${CMAKE_CURRENT_LIST_DIR})
