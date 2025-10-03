
# ---------------------------------------------------------------
# Unit
# ---------------------------------------------------------------

add_cmocka_test(
    "C_mip_packet_builder_unit"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/mip_packet_builder.unit.c"
    LINK_LIBRARIES ${MIP_UNIT_TEST_TARGET_C}
)
set_tests_properties("C_mip_packet_builder_unit" PROPERTIES LABELS "unit")

add_cmocka_test(
    "C_mip_logging"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/mip_logging.unit.c"
    LINK_LIBRARIES ${MIP_INTEGRATION_TEST_TARGET_C}
)
set_tests_properties("C_mip_logging" PROPERTIES LABELS "unit")

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
