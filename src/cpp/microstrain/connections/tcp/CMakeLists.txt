
# Add to existing C library.
target_sources(microstrain_socket
    PRIVATE
    tcp_connection.cpp
    tcp_connection.hpp
    ../connection.hpp
)

target_compile_features(microstrain_socket PUBLIC cxx_std_11)
