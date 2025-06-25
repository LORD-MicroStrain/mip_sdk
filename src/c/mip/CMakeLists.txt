set(MIP_LIBRARY "mip" CACHE INTERNAL "Name of the MIP library")
mark_as_advanced(MIP_LIBRARY)

include(microstrain_utilities)
microstrain_get_git_version(MIP_GIT_VERSION MIP_GIT_VERSION_CLEAN)
microstrain_extract_git_version(MIP_GIT_VERSION_CLEAN MIP_GIT_VERSION_MAJOR MIP_GIT_VERSION_MINOR MIP_GIT_VERSION_PATCH)

# Generate the version header file
set(MIP_VERSION_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/../../../cmake/mip_version.h.in")
set(MIP_VERSION_OUT_FILE "${CMAKE_CURRENT_LIST_DIR}/mip_version.h")
configure_file(${MIP_VERSION_IN_FILE} ${MIP_VERSION_OUT_FILE})

#
# Core MIP C library
#

set(MIP_SOURCES
    "${MIP_VERSION_OUT_FILE}"

    "${CMAKE_CURRENT_LIST_DIR}/mip_cmdqueue.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_cmdqueue.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_descriptors.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_descriptors.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_device_models.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_device_models.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_dispatch.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_dispatch.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_field.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_field.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_interface.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_interface.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_offsets.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_packet.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_packet.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_parser.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_parser.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_result.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_result.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_types.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_serialization.c"
    "${CMAKE_CURRENT_LIST_DIR}/mip_serialization.h"
    "${CMAKE_CURRENT_LIST_DIR}/definitions/common.c"
    "${CMAKE_CURRENT_LIST_DIR}/definitions/common.h"
    "${CMAKE_CURRENT_LIST_DIR}/mip_all.h"
)

add_library(${MIP_LIBRARY}
    "${MIP_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MIP_SOURCES})
endif()

target_compile_features(${MIP_LIBRARY} PUBLIC c_std_11)
target_link_libraries(${MIP_LIBRARY} PUBLIC ${MICROSTRAIN_LIBRARY})

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
list(TRANSFORM MIP_DEF_SOURCES PREPEND "${CMAKE_CURRENT_LIST_DIR}/definitions/")

if(MICROSTRAIN_CMAKE_DEBUG)
    set(INCLUDED_DEF_SOURCES ${MIP_DEF_SOURCES})
    list(TRANSFORM INCLUDED_DEF_SOURCES REPLACE "${CMAKE_CURRENT_LIST_DIR}/" "")
    message("C definitions = ${INCLUDED_DEF_SOURCES}")
endif()

target_sources(${MIP_LIBRARY} PRIVATE
    "${MIP_DEF_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MIP_DEF_SOURCES})
endif()

set(MIP_LIBRARIES_TMP ${MIP_LIBRARIES_TMP} ${MIP_LIBRARY} PARENT_SCOPE)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_library_install(${MIP_LIBRARY} ${MICROSTRAIN_SRC_DIR})
