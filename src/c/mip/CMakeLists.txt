

#option(MIP_ENABLE_DIAGNOSTICS "Enable various diagnostic counters in the mip library for debugging." OFF)

set(MIP_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")

set(MIP_SOURCES
    "${VERSION_OUT_FILE}"
    mip_cmdqueue.c
    mip_cmdqueue.h
    mip_descriptors.c
    mip_descriptors.h
    mip_dispatch.c
    mip_dispatch.h
    mip_field.c
    mip_field.h
    mip_interface.c
    mip_interface.h
    mip_offsets.h
    mip_packet.c
    mip_packet.h
    mip_parser.c
    mip_parser.h
    mip_result.c
    mip_result.h
    mip_types.h
    mip_serialization.c
    mip_serialization.h
    definitions/common.c
    definitions/common.h
    mip_all.h

    "utils/byte_ring.c"
    "utils/byte_ring.h"
)

add_library(mip ${MIP_SOURCES})
target_compile_features(mip PUBLIC c_std_11)
target_include_directories(mip INTERFACE ${MIP_INCLUDE_DIR})

set(MIPDEFS
    commands_3dm
    commands_aiding
    commands_base
    commands_filter
    commands_gnss
    commands_rtk
    commands_system
    data_filter
    data_gnss
    data_sensor
    data_shared
    data_system
)

# Set this list as a target property so the C++ version can access it.
set_target_properties(mip PROPERTIES definitions "${MIPDEFS}")

set(MIPDEF_HEADERS ${MIPDEFS})
set(MIPDEF_SOURCES ${MIPDEFS})
list(TRANSFORM MIPDEF_HEADERS APPEND ".h")
list(TRANSFORM MIPDEF_SOURCES APPEND ".c")
list(APPEND MIPDEF_SOURCES ${MIPDEF_HEADERS})
list(TRANSFORM MIPDEF_SOURCES PREPEND "definitions/")

if(MICROSTRAIN_CMAKE_DEBUG)
    message("C definitions = ${MIPDEF_SOURCES}")
endif()

target_sources(mip PRIVATE ${MIPDEF_SOURCES})



if(${MIP_ENABLE_DIAGNOSTICS})
    list(APPEND MIP_DEFINES "MIP_ENABLE_DIAGNOSTICS")
endif()

if(${MICROSTRAIN_TIMESTAMP_TYPE})
    list(APPEND MIP_DEFINES "MICROSTRAIN_TIMESTAMP_TYPE=${MICROSTRAIN_TIMESTAMP_TYPE}")
endif()

target_compile_definitions(mip PUBLIC "${MIP_DEFINES}")

target_link_libraries(mip PUBLIC microstrain_common)

#target_include_directories(mip INTERFACE ${CMAKE_CURRENT_LIST_DIR})
