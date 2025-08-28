
if(MICROSTRAIN_ENABLE_LOGGING)

    set(TEST_STRING_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/testutil_strings.h"
        "${CMAKE_CURRENT_LIST_DIR}/testutil_strings.c"
    )

    add_microstrain_test(
        NAME test_microstrain_string_concat
        SOURCES
            ${TEST_STRING_SOURCES}
            "${CMAKE_CURRENT_LIST_DIR}/test_strcat.c"
        COMMAND test_microstrain_string_concat
    )

    add_microstrain_test(
        NAME test_microstrain_string_formatting
        SOURCES
        ${TEST_STRING_SOURCES}
        "${CMAKE_CURRENT_LIST_DIR}/test_strfmt.c"
        COMMAND test_microstrain_string_formatting
    )

    add_microstrain_test(
        NAME test_microstrain_string_format_bytes
        SOURCES
        ${TEST_STRING_SOURCES}
        "${CMAKE_CURRENT_LIST_DIR}/test_strfmt_bytes.c"
        COMMAND test_microstrain_string_format_bytes
    )

endif()
