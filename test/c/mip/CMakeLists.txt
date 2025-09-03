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

# TODO: Disabled test is old and breaking in Jenkins. Should we keep it?
#[[
add_mip_test(
    NAME test_mip_parsing
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_parser.c"
    COMMAND test_mip_parsing
    ARGS "${MIP_TEST_DATA_BIN}"
)
]]

# TODO: Disabled test is old and takes a long time to run. Should we keep it?
#     ---> If we keep it, it should be moved to an integration test suite
#[[
add_mip_test(
    NAME test_mip_random
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_random.c"
    COMMAND test_mip_random
)

add_mip_test(
    NAME test_mip_packet_formatting
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/../microstrain/testutil_strings.c"
        "${CMAKE_CURRENT_LIST_DIR}/../microstrain/testutil_strings.h"
        "${CMAKE_CURRENT_LIST_DIR}/test_mip_format.c"
    COMMAND test_mip_packet_formatting
)
]]
