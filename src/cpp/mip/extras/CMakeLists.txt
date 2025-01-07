set(MIP_EXTRAS_LIBRARY "mip_extras" CACHE INTERNAL "Name of the MIP extras library")
mark_as_advanced(MIP_EXTRAS_LIBRARY)

set(MIP_EXTRAS_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/composite_result.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/descriptor_id.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/firmware_version.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/firmware_version.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/mip_extras_all.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/platform_connection.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/platform_connection.hpp"
)

add_library(${MIP_EXTRAS_LIBRARY}
    "${MIP_EXTRAS_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MIP_EXTRAS_SOURCES})
endif()

target_compile_features(${MIP_EXTRAS_LIBRARY} PUBLIC cxx_std_11)

target_link_libraries(${MIP_EXTRAS_LIBRARY} PUBLIC ${MIP_LIBRARY} ${MICROSTRAIN_EXTRAS_LIBRARY})

target_compile_definitions(${MIP_EXTRAS_LIBRARY}
    PUBLIC "MIP_ENABLE_EXTRAS=1"
    PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS}
)

target_compile_options(${MIP_EXTRAS_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_OPTIONS})

set(MIP_LIBRARIES_TMP ${MIP_LIBRARIES_TMP} ${MIP_EXTRAS_LIBRARY} PARENT_SCOPE)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_library_install(${MIP_EXTRAS_LIBRARY} ${MICROSTRAIN_SRC_DIR})
