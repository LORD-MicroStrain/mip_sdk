set(MIP_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")

set(MIP_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/mip.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_all.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_cmdqueue.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_descriptors.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_field.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_interface.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_interface.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_packet.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_parser.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_result.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_serialization.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/definitions/common.hpp"
    ${MIP_DEF_HEADERS}
)

target_sources(${MIP_LIBRARY} PRIVATE
    "${MIP_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MIP_SOURCES})
endif()

target_compile_features(${MIP_LIBRARY} PUBLIC cxx_std_11)
target_include_directories(${MIP_LIBRARY} PRIVATE ${MIP_INCLUDE_DIR})

# Get the mipdef list from the C version
get_target_property(MIP_DEFS ${MIP_LIBRARY} definitions)

set(MIP_DEF_HEADERS ${MIP_DEFS})
set(MIP_DEF_SOURCES ${MIP_DEFS})
list(TRANSFORM MIP_DEF_HEADERS APPEND ".hpp")
list(TRANSFORM MIP_DEF_SOURCES APPEND ".cpp")
list(APPEND MIP_DEF_SOURCES ${MIP_DEF_HEADERS})
list(TRANSFORM MIP_DEF_SOURCES PREPEND "${CMAKE_CURRENT_LIST_DIR}/definitions/")

if(MICROSTRAIN_CMAKE_DEBUG)
    set(INCLUDED_DEF_SOURCES ${MIP_DEF_SOURCES})
    list(TRANSFORM INCLUDED_DEF_SOURCES REPLACE "${CMAKE_CURRENT_LIST_DIR}/" "")
    message("C++ definitions = ${INCLUDED_DEF_SOURCES}")
endif()

target_sources(${MIP_LIBRARY} PRIVATE
    "${MIP_DEF_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MIP_DEF_SOURCES})
endif()

if(MIP_ENABLE_METADATA)
    add_subdirectory(metadata)
endif()

if(MIP_ENABLE_EXTRAS)
    add_subdirectory(extras)
endif()

set(MIP_LIBRARIES_TMP ${MIP_LIBRARIES_TMP} PARENT_SCOPE)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_install_headers(${MIP_LIBRARY} ${MICROSTRAIN_SRC_DIR})
