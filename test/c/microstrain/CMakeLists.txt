#target_sources(UnitTests PRIVATE
#    "strings.unit.c"
#)
#
#target_sources(IntegrationTests PRIVATE
#    # TODO: Add
#)

target_sources("${MIP_UNIT_TEST_TARGET_C}" PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/strings.unit.c"
)
