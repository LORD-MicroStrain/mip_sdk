
add_library(microstrain_serial
    "serial_port.h"
    "serial_port.c"
)

target_compile_features(microstrain_serial PUBLIC c_std_11)
target_compile_definitions(microstrain_serial PUBLIC "MICROSTRAIN_ENABLE_SERIAL=1")
target_link_libraries(microstrain_serial PUBLIC microstrain_common)
