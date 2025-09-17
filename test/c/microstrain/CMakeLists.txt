if(MIP_LOGGING_ENABLED)
    set(TEST_STRING_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/testutil_strings.h"
        "${CMAKE_CURRENT_LIST_DIR}/testutil_strings.c"
    )

    set(MICROSTRAIN_TEST_COMPILER_DEFINITIONS
        "MICROSTRAIN_LOGGING_MAX_LEVEL=${MICROSTRAIN_LOGGING_MAX_LEVEL}"
    )

    add_microstrain_test(
        NAME test_microstrain_string_concat
        SOURCES
            ${TEST_STRING_SOURCES}
            "${CMAKE_CURRENT_LIST_DIR}/test_strcat.c"
        COMMAND test_microstrain_string_concat
    )

    target_compile_definitions(test_microstrain_string_concat PRIVATE
        "${MICROSTRAIN_TEST_COMPILER_DEFINITIONS}"
    )

    add_microstrain_test(
        NAME test_microstrain_string_formatting
        SOURCES
        ${TEST_STRING_SOURCES}
        "${CMAKE_CURRENT_LIST_DIR}/test_strfmt.c"
        COMMAND test_microstrain_string_formatting
    )

    target_compile_definitions(test_microstrain_string_formatting PRIVATE
        "${MICROSTRAIN_TEST_COMPILER_DEFINITIONS}"
    )

    add_microstrain_test(
        NAME test_microstrain_string_format_bytes
        SOURCES
        ${TEST_STRING_SOURCES}
        "${CMAKE_CURRENT_LIST_DIR}/test_strfmt_bytes.c"
        COMMAND test_microstrain_string_format_bytes
    )

    target_compile_definitions(test_microstrain_string_format_bytes PRIVATE
        "${MICROSTRAIN_TEST_COMPILER_DEFINITIONS}"
    )
endif()
