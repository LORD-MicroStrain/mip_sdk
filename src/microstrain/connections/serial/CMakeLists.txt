

set(SOURCES
    serial_connection.cpp
    serial_connection.hpp
    serial_port.h
    serial_port.c
    ../connection.hpp
)

add_library(microstrain_serial ${SOURCES})

target_link_libraries(microstrain_serial PUBLIC microstrain_common)

#target_include_directories(microstrain_serial PRIVATE ${SRC_DIR})
