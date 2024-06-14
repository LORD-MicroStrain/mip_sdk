
set(SOURCES
    tcp_socket.h
    tcp_socket.c
)

add_library(microstrain_socket ${SOURCES})

target_link_libraries(microstrain_socket PUBLIC microstrain_common)

if(WIN32)
    target_link_libraries(microstrain_socket PUBLIC "ws2_32")
endif()
