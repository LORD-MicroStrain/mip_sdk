
option(WITH_SOCKETS "Use the included socket library." OFF)

if(WITH_SOCKETS)

    set(SOCKET_DIR "${CMAKE_CURRENT_LIST_DIR}")

    set(SOCKET_SOURCES
        "${SOCKET_DIR}/tcp.cpp"
        "${SOCKET_DIR}/tcp.hpp"
    )

    add_library(socket ${SOCKET_SOURCES})
    target_include_directories(socket PUBLIC "${EXAMPLE_DIR}")

endif(WITH_SOCKETS)
