add_mip_test(
    NAME TestMipFields
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_fields.c"
    COMMAND TestMipFields
)

add_mip_test(
    NAME TestMipPacketBuilding
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_packet_builder.c"
    COMMAND TestMipPacketBuilding
)

add_mip_test(
    NAME TestMipParsing
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_parser.c"
    COMMAND TestMipParsing
    ARGS "${MIP_TEST_DATA_BIN}"
)

add_mip_test(
    NAME TestMipRandom
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_random.c"
    COMMAND TestMipRandom
)
