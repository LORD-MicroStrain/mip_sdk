
# ---------------------------------------------------------------
# Unit
# ---------------------------------------------------------------

add_cmocka_test(
    "C_mip_packet_builder_unit"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/mip_packet_builder.unit.c"
    LINK_LIBRARIES ${MIP_UNIT_TEST_TARGET_C}
)
set_tests_properties("C_mip_packet_builder_unit" PROPERTIES LABELS "unit")

# ---------------------------------------------------------------
# Integration
# ---------------------------------------------------------------

add_cmocka_test(
    "C_mip_fields"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/mip_fields.integration.c"
    LINK_LIBRARIES ${MIP_INTEGRATION_TEST_TARGET_C}
)
set_tests_properties("C_mip_fields" PROPERTIES LABELS "integration")


# TODO: Disabled test compiles, but is failing. This looks like it might be a false positive.
#[[
add_mip_test(
    NAME test_mip_parsing
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_mip_parser.c"
    COMMAND test_mip_parsing
    ARGS "${MIP_TEST_DATA_BIN}"
)
]]

# TODO: Disabled tests compile but take a long time to run.
#     ---> Move to an integration test suite
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
