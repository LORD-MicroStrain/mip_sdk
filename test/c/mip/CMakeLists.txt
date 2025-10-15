target_sources("${MIP_UNIT_TEST_TARGET_C}" PRIVATE
    "packet_builder.unit.c"
    "logging.unit.c"
    "parser.integration.c"
)

target_sources("${MIP_INTEGRATION_TEST_TARGET_C}" PRIVATE
    #"parser.integration.c"
)
