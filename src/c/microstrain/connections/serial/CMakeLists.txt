set(MICROSTRAIN_SERIAL_LIBRARY "microstrain_serial" CACHE INTERNAL "Name of the MicroStrain serial connections library")
mark_as_advanced(MICROSTRAIN_SERIAL_LIBRARY)

add_library(${MICROSTRAIN_SERIAL_LIBRARY}
    "serial_port.h"
    "serial_port.c"
)

target_compile_features(${MICROSTRAIN_SERIAL_LIBRARY} PUBLIC c_std_11)

target_link_libraries(${MICROSTRAIN_SERIAL_LIBRARY} PUBLIC ${MICROSTRAIN_COMMON_LIBRARY})

target_compile_definitions(${MICROSTRAIN_SERIAL_LIBRARY}
    PUBLIC "MICROSTRAIN_ENABLE_SERIAL=1"
    PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS}
)

target_compile_options(${MICROSTRAIN_SERIAL_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_OPTIONS})

target_link_libraries(${MICROSTRAIN_LIBRARIES} INTERFACE ${MICROSTRAIN_SERIAL_LIBRARY})

#
# Installation
#
include(microstrain_utilities)
microstrain_setup_library_install(${MICROSTRAIN_SERIAL_LIBRARY} ${MICROSTRAIN_COMMON_LIBRARY})
