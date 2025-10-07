# ---------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------

add_cmocka_test(
    "C_Strings"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/strings.unit.c"
    LINK_LIBRARIES ${MIP_UNIT_TEST_TARGET_C}
)
set_tests_properties("C_Strings" PROPERTIES LABELS "unit")
