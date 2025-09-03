# TODO: The commented out tests are broken

#[[
add_microstrain_test(
    NAME TestMicroStrainSerialConnection
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_serial.cpp"
    COMMAND TestMicroStrainSerialConnection
    ADDL_LINK_LIBRARIES ${MICROSTRAIN_SERIAL_LIBRARY}
    USE_CPP TRUE
)
]]

add_microstrain_test(
    NAME TestMicroStrainStrings
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/../../c/microstrain/testutil_strings.c"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/../../c/microstrain/testutil_strings.h"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_strings.cpp"
    COMMAND TestMicroStrainStrings
    USE_CPP TRUE
)
