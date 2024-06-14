

target_sources(microstrain_serial
    PRIVATE
    serial_connection.cpp
    PUBLIC
    serial_connection.hpp
    ../connection.hpp
)
