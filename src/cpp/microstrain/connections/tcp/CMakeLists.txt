

target_sources(microstrain_socket
    PRIVATE
    tcp_connection.cpp
    PUBLIC
    tcp_connection.hpp
    ../connection.hpp
)

target_compile_features(microstrain_socket PUBLIC cxx_std_11)
target_include_directories(microstrain_socket INTERFACE ${MICROSTRAIN_INCLUDE_DIR})

