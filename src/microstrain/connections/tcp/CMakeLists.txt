

set(SOURCES
    tcp_connection.cpp
    tcp_connection.hpp
    tcp_socket.h
    tcp_socket.c
    ../connection.hpp
)

add_library(microstrain_socket ${SOURCES})

target_link_libraries(microstrain_socket PUBLIC microstrain_common)

if(WIN32)
    target_link_libraries(microstrain_socket PUBLIC "ws2_32")
endif()

target_include_directories(microstrain_socket PRIVATE ${SRC_DIR})
