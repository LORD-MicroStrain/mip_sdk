# ---------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------

add_cmocka_test(
    "C_mip_packet_builder_unit"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/packet_builder.unit.c"
    LINK_LIBRARIES ${MIP_UNIT_TEST_TARGET_C}
)
set_tests_properties("C_mip_packet_builder_unit" PROPERTIES LABELS "unit")

add_cmocka_test(
    "C_mip_logging"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/logging.unit.c"
    LINK_LIBRARIES ${MIP_INTEGRATION_TEST_TARGET_C}
)
set_tests_properties("C_mip_logging" PROPERTIES LABELS "unit")

# ---------------------------------------------------------------
# Integration tests
# ---------------------------------------------------------------

add_cmocka_test(
    "C_mip_parser_integration"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/parser.integration.c"
    LINK_LIBRARIES ${MIP_UNIT_TEST_TARGET_C}
)
set_tests_properties("C_mip_parser_integration"
    PROPERTIES
        LABELS "SKIP;integration"
        DISABLED TRUE
)
