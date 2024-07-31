
set(SOURCES
    serial_port.h
    serial_port.c
)

add_library(microstrain_serial ${SOURCES})
target_compile_features(microstrain_serial PUBLIC c_std_11)

target_link_libraries(microstrain_serial PUBLIC microstrain_common)
