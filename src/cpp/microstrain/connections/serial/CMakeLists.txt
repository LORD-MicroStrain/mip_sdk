set(MICROSTRAIN_SERIAL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/serial_connection.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/serial_connection.hpp"
    "${MICROSTRAIN_CONNECTION_SOURCES}"
)

# Add to existing C library.
target_sources(${MICROSTRAIN_SERIAL_LIBRARY} PRIVATE
    "${MICROSTRAIN_SERIAL_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MICROSTRAIN_SERIAL_SOURCES})
endif()

target_compile_features(${MICROSTRAIN_SERIAL_LIBRARY} PUBLIC cxx_std_11)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_install_headers(${MICROSTRAIN_SERIAL_LIBRARY} ${MICROSTRAIN_SRC_DIR})
