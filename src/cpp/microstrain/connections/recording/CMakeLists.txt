set(MICROSTRAIN_RECORDING_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/recording_connection.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/recording_connection.hpp"
    "${MICROSTRAIN_CONNECTION_SOURCES}"
)

# Add to existing C library.
target_sources(${MICROSTRAIN_RECORDING_LIBRARY} PRIVATE
    "${MICROSTRAIN_RECORDING_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MICROSTRAIN_RECORDING_SOURCES})
endif()

target_compile_features(${MICROSTRAIN_RECORDING_LIBRARY} PUBLIC cxx_std_11)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_library_install(${MICROSTRAIN_RECORDING_LIBRARY} ${MICROSTRAIN_SRC_DIR})
