# TODO: Add doc/note for this file
# TODO: Add docs for functions

function(internal_parse_test_registrations_from_sources
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


# Generate logic to conditionally run each test (based on commandline filtering).
function(internal_generate_test_execution_logic_with_commandline_filtering
    DISCOVERED_SUITES
    DISCOVERED_TESTS
    TEST_FILEPATHS
    OUT_MAIN_CONTENT
)
    # Get current content
    set(MAIN_CONTENT "${${OUT_MAIN_CONTENT}}")

    list(LENGTH DISCOVERED_TESTS TEST_COUNT)
    math(EXPR LAST_INDEX "${TEST_COUNT} - 1")

    foreach(INDEX RANGE ${LAST_INDEX})
        # We need to set the filepath explicitly (for Unity) since it points to the generated
        # runner file instead.
        list(GET TEST_FILEPATHS ${INDEX} TEST_FILEPATH)
        list(GET DISCOVERED_SUITES ${INDEX} SUITE_NAME)
        list(GET DISCOVERED_TESTS ${INDEX} TEST_NAME)

        # Escape backslashes in filepaths for C string literal
        string(REPLACE "\\" "\\\\" TEST_FILEPATH_ESCAPED "${TEST_FILEPATH}")

        set(MAIN_CONTENT "${MAIN_CONTENT}    if (!test_filter || strcmp(test_filter, \"[${SUITE_NAME}] ${TEST_NAME}\") == 0) {\n")
        set(MAIN_CONTENT "${MAIN_CONTENT}        INTERNAL_RUN_MICROSTRAIN_TEST_CASE_AUTO_DISCOVER(${SUITE_NAME}, ${TEST_NAME}, \"${TEST_FILEPATH_ESCAPED}\");\n")
        set(MAIN_CONTENT "${MAIN_CONTENT}    }\n")
    endforeach()
endfunction()


# Creates an empty test runner file (populate this file later).
function(internal_create_test_runner_file TARGET_NAME OUT_GENERATED_FILEPATH)
    # Create directory for the generated file
    set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_tests/${TARGET_NAME}")
    file(MAKE_DIRECTORY "${GENERATED_DIR}")

    # Create the file
    set(GENERATED_MAIN "${GENERATED_DIR}/run_tests.c")

    # Return the filepath to parent scope
    set(${OUT_GENERATED_FILEPATH} "${GENERATED_MAIN}" PARENT_SCOPE)
endfunction()


# Used when no tests are discovered to avoid a failing build.
function(internal_generate_test_runner_file_with_dummy_main TARGET_NAME)
    internal_create_test_runner_file("${TARGET_NAME}" GENERATED_MAIN)

    # Write empty main
    file(WRITE "${GENERATED_MAIN}" "int main(void) { return 0; }\n")

    # Add to target
    target_sources(${TARGET_NAME} PRIVATE "${GENERATED_MAIN}")
endfunction()


# Generate a main runner file for all tests. Allows all tests to be run at once, or filtered
# through the command-line.
function(internal_generate_test_runner_file_with_main
   TARGET_NAME
   DISCOVERED_SUITES
   DISCOVERED_TESTS
   TEST_FILEPATHS
   OUT_GENERATED_MAIN
)
    internal_create_test_runner_file("${TARGET_NAME}" GENERATED_MAIN)

    # Generate includes and definitions
    set(MAIN_CONTENT "/* Auto-generated test main for ${TARGET_NAME}*/\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}#include <string.h>\n\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}#include <microstrain_test/microstrain_test.h>\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}MICROSTRAIN_TEST_DEFAULT_SETUP();\n")

    # Generate test case declarations
    list(LENGTH DISCOVERED_TESTS TEST_COUNT)
    math(EXPR LAST_INDEX "${TEST_COUNT} - 1")
    foreach(INDEX RANGE ${LAST_INDEX})
        list(GET DISCOVERED_SUITES ${INDEX} SUITE_NAME)
        list(GET DISCOVERED_TESTS ${INDEX} TEST_NAME)
        set(MAIN_CONTENT "${MAIN_CONTENT}extern MICROSTRAIN_TEST_CASE(${SUITE_NAME}, ${TEST_NAME});\n")
    endforeach()

    # Generate main function with commandline argument parsing for test filtering.
    set(MAIN_CONTENT "${MAIN_CONTENT}\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}int main(int argc, char** argv) {\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    const char* test_filter = NULL;\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    \n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    // Parse command line for --test=name\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    for (int i = 1; i < argc; ++i) {\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}        if (strncmp(argv[i], \"--test=\", 7) == 0) {\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}            test_filter = argv[i] + 7;\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}            break;\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}        }\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    }\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    \n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    (void)test_filter; /* Suppress unused variable warning if only one test */\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    \n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    MICROSTRAIN_TEST_BEGIN();\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    \n")

    internal_generate_test_execution_logic_with_commandline_filtering(
        "${DISCOVERED_SUITES}"
        "${DISCOVERED_TESTS}"
        "${TEST_FILEPATHS}"
        MAIN_CONTENT
    )

    # Generate the end of the main function.
    set(MAIN_CONTENT "${MAIN_CONTENT}    \n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    return MICROSTRAIN_TEST_END();\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}}\n")

    # Write the generated contents to the main file and return the filepath to the parent scope.
    file(WRITE "${GENERATED_MAIN}" "${MAIN_CONTENT}")
    set(${OUT_GENERATED_MAIN} "${GENERATED_MAIN}" PARENT_SCOPE)
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
