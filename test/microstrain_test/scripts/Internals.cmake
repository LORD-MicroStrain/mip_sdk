# TODO: Add doc/note for this file
# TODO: Add docs for functions

function(internal_parse_tests_from_sources
    TARGET_SOURCES
    OUT_SUITES
    OUT_TESTS
    OUT_FILEPATHS
)
    set(DISCOVERED_SUITES "")
    set(DISCOVERED_TESTS "")
    set(TEST_FILEPATHS "")

    foreach(SOURCE_FILE ${TARGET_SOURCES})
        get_filename_component(SOURCE_FILE_ABSOLUTE_PATH "${SOURCE_FILE}" ABSOLUTE)

        # Set CMake to reload when source changes to discover newly added tests.
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SOURCE_FILE_ABSOLUTE_PATH}")

        if(NOT EXISTS "${SOURCE_FILE_ABSOLUTE_PATH}")
            continue()
        endif()

        file(READ "${SOURCE_FILE_ABSOLUTE_PATH}" FILE_CONTENT)

        # Remove block comments /* ... */
        # This regex handles multiline block comments. It also handles complex cases such as if
        # asterisks are inside the comment, etc.
        string(REGEX REPLACE "/\\*([^*]|\\*+[^*/])*\\*+/" "" FILE_CONTENT "${FILE_CONTENT}")

        # Split file into lines to check for line comments.
        # Replaces every ";" with "\\;" (so they aren't interpreted as the CMake list separator).
        string(REGEX REPLACE ";" "\\\\;" FILE_CONTENT "${FILE_CONTENT}")
        # Converts the file to a CMake list where each element is one line by replacing newlines
        # with the CMake list separator. This allows the lines to be looped over.
        string(REGEX REPLACE "\n" ";" FILE_LINES "${FILE_CONTENT}")

        foreach(LINE ${FILE_LINES})
            # Remove everything after // (line comments)
            string(REGEX REPLACE "//.*$" "" LINE_WITHOUT_COMMENT "${LINE}")

            # Check if the line contains a test registration (after comment removal).
            string(REGEX MATCH "MICROSTRAIN_TEST_CASE\\([^)]+\\)" MATCH "${LINE_WITHOUT_COMMENT}")

            if(MATCH)
                # Extract the test name and test suite.
                string(REGEX REPLACE "MICROSTRAIN_TEST_CASE\\(([a-zA-Z0-9_]+)[ \t]*,[ \t]*([a-zA-Z0-9_]+)\\)" "\\1" SUITE_NAME "${MATCH}")
                string(REGEX REPLACE "MICROSTRAIN_TEST_CASE\\(([a-zA-Z0-9_]+)[ \t]*,[ \t]*([a-zA-Z0-9_]+)\\)" "\\2" TEST_NAME "${MATCH}")

                list(APPEND DISCOVERED_SUITES ${SUITE_NAME})
                list(APPEND DISCOVERED_TESTS ${TEST_NAME})
                list(APPEND TEST_FILEPATHS "${SOURCE_FILE_ABSOLUTE_PATH}")
            endif()
        endforeach()
    endforeach()

    # Return results to parent scope
    set(${OUT_SUITES} ${DISCOVERED_SUITES} PARENT_SCOPE)
    set(${OUT_TESTS} ${DISCOVERED_TESTS} PARENT_SCOPE)
    set(${OUT_FILEPATHS} ${TEST_FILEPATHS} PARENT_SCOPE)
endfunction()


# Register each test with CTest
function(internal_register_individual_tests_with_ctest
    TARGET_NAME
    DISCOVERED_SUITES
    DISCOVERED_TESTS
    LABELS
    DISABLED
    SEQUENTIAL
)
    list(LENGTH DISCOVERED_TESTS TEST_COUNT)
    math(EXPR LAST_INDEX "${TEST_COUNT} - 1")

    foreach(INDEX RANGE ${LAST_INDEX})
        list(GET DISCOVERED_SUITES ${INDEX} SUITE_NAME)
        list(GET DISCOVERED_TESTS  ${INDEX} TEST_NAME)

        # Set how the test name should be displayed in the console when running tests.
        # Currently in this format: "[Example_suite] Example_test"
        set(TEST_DISPLAY "[${SUITE_NAME}] ${TEST_NAME}")

        add_test(NAME ${TEST_DISPLAY} COMMAND ${TARGET_NAME} --test=${TEST_DISPLAY})

        set_tests_properties(${TEST_DISPLAY}
            PROPERTIES
                TIMEOUT 30
                LABELS "${TARGET_NAME};${SUITE_NAME}"
        )

        if(LABELS)
            set_tests_properties(${TEST_DISPLAY} PROPERTIES LABELS ${LABELS})
        endif()

        if(DISABLED)
            set_tests_properties(${TEST_DISPLAY} PROPERTIES DISABLED TRUE)
        endif()

        if(SEQUENTIAL)
            set_tests_properties(${TEST_DISPLAY} PROPERTIES RESOURCE_LOCK ${TARGET_NAME})
        endif()
    endforeach()
endfunction()
