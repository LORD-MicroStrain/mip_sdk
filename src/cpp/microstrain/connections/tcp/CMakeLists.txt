set(MICROSTRAIN_SOCKET_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/tcp_connection.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/tcp_connection.hpp"
    "${MICROSTRAIN_CONNECTION_SOURCES}"
)

# Add to existing C library.
target_sources(${MICROSTRAIN_SOCKET_LIBRARY} PRIVATE
    "${MICROSTRAIN_SOCKET_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MICROSTRAIN_SOCKET_SOURCES})
endif()

target_compile_features(${MICROSTRAIN_SOCKET_LIBRARY} PUBLIC cxx_std_11)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_install_headers(${MICROSTRAIN_SOCKET_LIBRARY} ${MICROSTRAIN_SRC_DIR})
