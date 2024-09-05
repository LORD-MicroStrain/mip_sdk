# Add to existing C library.
target_sources(${MICROSTRAIN_SERIAL_LIBRARY} PRIVATE
    serial_connection.cpp
    serial_connection.hpp
    ../connection.hpp
)

target_compile_features(${MICROSTRAIN_SERIAL_LIBRARY} PUBLIC cxx_std_11)

#
# Installation
#
include(microstrain_utilities)
microstrain_setup_install_headers(${MICROSTRAIN_SERIAL_LIBRARY})
