
add_library(microstrain_socket
    "tcp_socket.h"
    "tcp_socket.c"
)

target_link_libraries(microstrain_socket PRIVATE microstrain_private)
target_compile_features(microstrain_socket PUBLIC c_std_11)
target_compile_definitions(microstrain_socket PUBLIC "MICROSTRAIN_ENABLE_TCP=1")
target_link_libraries(microstrain_socket PUBLIC microstrain_common)

if(WIN32)
    target_link_libraries(microstrain_socket PUBLIC "ws2_32")
endif()
