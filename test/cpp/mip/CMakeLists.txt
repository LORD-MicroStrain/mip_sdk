add_mip_test(
    NAME TestMipPerf
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/mip_parser_performance.cpp"
    COMMAND TestMipPerf
    USE_CPP TRUE
)

add_mip_test(
    NAME TestMipCpp
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_mip.cpp"
    COMMAND TestMipCpp
    USE_CPP TRUE
)

add_mip_test(
    NAME TestMipPacket
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_packet_interface.cpp"
    COMMAND TestMipPacket
    USE_CPP TRUE
)
