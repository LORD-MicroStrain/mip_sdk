# ---------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------

add_unity_test(
    NAME "C_Strings"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/strings.unit.c"
    TARGET ${MIP_UNIT_TEST_TARGET_C}
    LABELS "unit"
)
