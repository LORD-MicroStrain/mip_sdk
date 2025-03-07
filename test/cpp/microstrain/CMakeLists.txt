# TODO: Fix test with V3 changes
add_microstrain_test(
    NAME TestMicroStrainSerialConnection
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_serial.cpp"
    COMMAND TestMicroStrainSerialConnection
    ADDL_LINK_LIBRARIES ${MICROSTRAIN_SERIAL_LIBRARY}
    USE_CPP TRUE
)
