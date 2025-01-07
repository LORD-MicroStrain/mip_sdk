set(MIP_METADATA_LIBRARY "mip_metadata" CACHE INTERNAL "Name of the MIP metadata library")
mark_as_advanced(MIP_METADATA_LIBRARY)

# Get the mipdef list from the C version
get_target_property(MIP_DEFS ${MIP_LIBRARY} definitions)

set(MIP_DEF_HEADERS ${MIP_DEFS})
list(TRANSFORM MIP_DEF_HEADERS APPEND ".hpp")
list(TRANSFORM MIP_DEF_HEADERS PREPEND "${CMAKE_CURRENT_LIST_DIR}/definitions/")

if(MICROSTRAIN_CMAKE_DEBUG)
    set(INCLUDED_DEF_HEADERS ${MIP_DEF_HEADERS})
    list(TRANSFORM INCLUDED_DEF_HEADERS REPLACE "${CMAKE_CURRENT_LIST_DIR}/" "")
   message("C++ metadata definitions = ${INCLUDED_DEF_HEADERS}")
endif()

set(MIP_METADATA_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/mip_all_definitions.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_definitions.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_definitions.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_metadata.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_structures.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_meta_utils.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/definitions/common.hpp"
    ${MIP_DEF_HEADERS}
)

add_library(${MIP_METADATA_LIBRARY}
    "${MIP_METADATA_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MIP_METADATA_SOURCES})
endif()

target_compile_features(${MIP_METADATA_LIBRARY} PUBLIC cxx_std_20)

target_link_libraries(${MIP_METADATA_LIBRARY} PUBLIC ${MIP_LIBRARY})

target_compile_definitions(${MIP_METADATA_LIBRARY}
    PUBLIC "MIP_ENABLE_METADATA=1"
    PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    list(APPEND MICROSTRAIN_PRIVATE_COMPILE_OPTIONS
        "-Wno-missing-braces" # Suppress missing braces warning
    )
endif()

target_compile_options(${MIP_METADATA_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_OPTIONS})

set(MIP_LIBRARIES_TMP ${MIP_LIBRARIES_TMP} ${MIP_METADATA_LIBRARY} PARENT_SCOPE)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_library_install(${MIP_METADATA_LIBRARY} ${MICROSTRAIN_SRC_DIR})
