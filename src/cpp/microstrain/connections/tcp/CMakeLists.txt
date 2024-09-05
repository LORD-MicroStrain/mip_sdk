# Add to existing C library.
target_sources(${MICROSTRAIN_SOCKET_LIBRARY} PRIVATE
    tcp_connection.cpp
    tcp_connection.hpp
    ../connection.hpp
)

target_compile_features(${MICROSTRAIN_SOCKET_LIBRARY} PUBLIC cxx_std_11)

#
# Installation
#
include(microstrain_utilities)
microstrain_setup_install_headers(${MICROSTRAIN_SOCKET_LIBRARY})
