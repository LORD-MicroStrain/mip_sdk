
if(MICROSTRAIN_ENABLE_LOGGING)

    set(TEST_STRING_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/test_strings.h"
        "${CMAKE_CURRENT_LIST_DIR}/test_strings.c"
    )

    add_microstrain_test(
        NAME test_microstrain_strcat
        SOURCES
            ${TEST_STRING_SOURCES}
            "${CMAKE_CURRENT_LIST_DIR}/test_strcat.c"
        COMMAND test_microstrain_strcat
    )

    add_microstrain_test(
        NAME test_microstrain_strfmt
        SOURCES
        ${TEST_STRING_SOURCES}
        "${CMAKE_CURRENT_LIST_DIR}/test_strfmt.c"
        COMMAND test_microstrain_strfmt
    )

endif()
