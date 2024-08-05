

target_sources(microstrain_serial
    PRIVATE
    serial_connection.cpp
    serial_connection.hpp
    ../connection.hpp
)

target_compile_features(microstrain_serial PUBLIC cxx_std_11)
