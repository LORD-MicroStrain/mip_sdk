
set(SOURCES
    tcp_socket.h
    tcp_socket.c
)

add_library(microstrain_socket ${SOURCES})
target_compile_features(microstrain_socket PUBLIC c_std_11)

target_include_directories(microstrain_socket INTERFACE ${MICROSTRAIN_INCLUDE_DIR})

target_link_libraries(microstrain_socket PUBLIC microstrain_common)

if(WIN32)
    target_link_libraries(microstrain_socket PUBLIC "ws2_32")
endif()
