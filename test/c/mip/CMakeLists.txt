# ---------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------

add_unity_test(
    NAME "C_mip_packet_builder_unit"
    SOURCES "packet_builder.unit.c"
    TARGET ${MIP_UNIT_TEST_TARGET_C}
    LABELS "unit"
)

add_unity_test(
    NAME "C_mip_logging"
    SOURCES "logging.unit.c"
    TARGET ${MIP_UNIT_TEST_TARGET_C}
    LABELS "unit"
    DISABLED
)

# ---------------------------------------------------------------
# Integration tests
# ---------------------------------------------------------------

add_unity_test(
    NAME "C_mip_parser_integration"
    SOURCES "parser.integration.c"
    TARGET ${MIP_UNIT_TEST_TARGET_C}
    LABELS "integration"
    DISABLED
)
