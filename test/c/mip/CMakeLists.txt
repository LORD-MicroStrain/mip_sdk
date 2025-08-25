add_mip_test(
    NAME test_mip_fields
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_fields.c"
    COMMAND test_mip_fields
)

add_mip_test(
    NAME test_mip_packet_builder
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_packet_builder.c"
    COMMAND test_mip_packet_builder
)

add_mip_test(
    NAME test_mip_parsing
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_parser.c"
    COMMAND test_mip_parsing
    ARGS "${MIP_TEST_DATA_BIN}"
)

add_mip_test(
    NAME test_mip_random
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_random.c"
    COMMAND test_mip_random
)

add_mip_test(
    NAME TestMipFormat
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/../microstrain/test_strings.c"
        "${CMAKE_CURRENT_LIST_DIR}/../microstrain/test_strings.h"
        "${CMAKE_CURRENT_LIST_DIR}/test_mip_format.c"
    COMMAND TestMipFormat
)
