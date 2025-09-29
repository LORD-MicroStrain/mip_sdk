add_cmocka_test(
    "mip_strings_unit_test_c"
    SOURCES ${CMAKE_CURRENT_LIST_DIR}/strings.unit.c
    LINK_LIBRARIES ${MIP_UNIT_TEST_TARGET_C}
)
