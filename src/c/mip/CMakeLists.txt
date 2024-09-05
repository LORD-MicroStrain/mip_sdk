set(MIP_LIBRARY "mip" CACHE INTERNAL "Name of the MIP library")
mark_as_advanced(MIP_LIBRARY)

#
# Core MIP C library
#

add_library(${MIP_LIBRARY}
    "${VERSION_OUT_FILE}"

    "mip_cmdqueue.c"
    "mip_cmdqueue.h"
    "mip_descriptors.c"
    "mip_descriptors.h"
    "mip_device_models.c"
    "mip_device_models.h"
    "mip_dispatch.c"
    "mip_dispatch.h"
    "mip_field.c"
    "mip_field.h"
    "mip_interface.c"
    "mip_interface.h"
    "mip_offsets.h"
    "mip_packet.c"
    "mip_packet.h"
    "mip_parser.c"
    "mip_parser.h"
    "mip_result.c"
    "mip_result.h"
    "mip_types.h"
    "mip_serialization.c"
    "mip_serialization.h"
    "definitions/common.c"
    "definitions/common.h"
    "mip_all.h"

    "utils/byte_ring.c"
    "utils/byte_ring.h"
)

target_compile_features(${MIP_LIBRARY} PUBLIC c_std_11)
target_link_libraries(${MIP_LIBRARY} PUBLIC ${MICROSTRAIN_COMMON_LIBRARY})

if(${MIP_ENABLE_DIAGNOSTICS})
    target_compile_definitions(${MIP_LIBRARY} PUBLIC "MIP_ENABLE_DIAGNOSTICS=1")
endif()

target_compile_definitions(${MIP_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS})

target_compile_options(${MIP_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_OPTIONS})


#
# Mip definition files
#

set(MIP_DEFS
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
set_target_properties(${MIP_LIBRARY} PROPERTIES definitions "${MIP_DEFS}")

set(MIP_DEF_HEADERS ${MIP_DEFS})
set(MIP_DEF_SOURCES ${MIP_DEFS})
list(TRANSFORM MIP_DEF_HEADERS APPEND ".h")
list(TRANSFORM MIP_DEF_SOURCES APPEND ".c")
list(APPEND MIP_DEF_SOURCES ${MIP_DEF_HEADERS})
list(TRANSFORM MIP_DEF_SOURCES PREPEND "definitions/")

if(MICROSTRAIN_CMAKE_DEBUG)
    message("C definitions = ${MIP_DEF_SOURCES}")
endif()

target_sources(${MIP_LIBRARY} PRIVATE ${MIP_DEF_SOURCES})

target_link_libraries(${MIP_LIBRARIES} INTERFACE ${MIP_LIBRARY})

#
# Installation
#
include(microstrain_utilities)
microstrain_setup_library_install(${MIP_LIBRARY} ${MIP_LIBRARY})
