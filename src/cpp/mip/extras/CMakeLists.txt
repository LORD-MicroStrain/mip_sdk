set(MIP_EXTRAS_LIBRARY "mip_extras" CACHE INTERNAL "Name of the MIP extras library")
mark_as_advanced(MIP_EXTRAS_LIBRARY)

set(UTILS_SUBDIR_TIME "${CMAKE_CURRENT_LIST_DIR}/time")
add_library(${MIP_EXTRAS_LIBRARY}
    "${CMAKE_CURRENT_LIST_DIR}/composite_result.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/descriptor_id.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/firmware_version.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/firmware_version.hpp"

    "${UTILS_SUBDIR_TIME}/durations.hpp"
    "${UTILS_SUBDIR_TIME}/misc.hpp"
    "${UTILS_SUBDIR_TIME}/standards.cpp"
    "${UTILS_SUBDIR_TIME}/standards.hpp"
    "${UTILS_SUBDIR_TIME}/timestamp.cpp"
    "${UTILS_SUBDIR_TIME}/timestamp.hpp"
)

target_compile_features(${MIP_EXTRAS_LIBRARY} PUBLIC cxx_std_11)

target_link_libraries(${MIP_EXTRAS_LIBRARY} PUBLIC ${MIP_LIBRARY} ${MICROSTRAIN_EXTRAS_LIBRARY})

target_compile_definitions(${MIP_EXTRAS_LIBRARY}
    PUBLIC "MIP_ENABLE_EXTRAS=1"
    PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS}
)

target_compile_options(${MIP_EXTRAS_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_OPTIONS})

target_link_libraries(${MIP_LIBRARIES} INTERFACE ${MIP_EXTRAS_LIBRARY})

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_library_install(${MIP_EXTRAS_LIBRARY} ${MICROSTRAIN_SRC_DIR})
