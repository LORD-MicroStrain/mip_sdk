

target_sources(microstrain_socket
    PRIVATE
    tcp_connection.cpp
    PUBLIC
    tcp_connection.hpp
    ../connection.hpp
)
