# ---------------------------------------------------------------
# Utility
# ---------------------------------------------------------------

# TODO: Replace with automatic test discovery when implemented

# Temporary function that adds a CTest test
function(add_unity_test)
    cmake_parse_arguments(
        ARG
        ""
        "NAME"
        "SOURCES;TARGET"
        ${ARGN}
    )
    add_executable(${ARG_NAME} ${ARG_SOURCES})
    target_link_libraries(${ARG_NAME} PRIVATE ${ARG_TARGET})
    add_test(NAME ${ARG_NAME} COMMAND ${ARG_NAME})
endfunction()

# ---------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------

add_unity_test(
    NAME "C_Strings"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/strings.unit.c"
    TARGET ${MIP_UNIT_TEST_TARGET_C}
)
set_tests_properties("C_Strings" PROPERTIES LABELS "unit")

#[[
add_cmocka_test(
    "C_Strings"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/strings.unit.c"
    LINK_LIBRARIES ${MIP_UNIT_TEST_TARGET_C}
)
set_tests_properties("C_Strings" PROPERTIES LABELS "unit")
]]
