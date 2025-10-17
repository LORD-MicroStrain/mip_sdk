# ---------------------------------------------------------------
# C discovery
# ---------------------------------------------------------------

function(microstrain_discover_tests_c)
    cmake_parse_arguments(
        ARG
        "SEQUENTIAL"
        "TARGET"
        "LABELS"
        ${ARGN}
    )

    # TODO: Maybe split script part into separate utility functions

    # Verify the target exists
    if(NOT TARGET ${ARG_TARGET})
        message(FATAL_ERROR "Target ${ARG_TARGET} does not exist")
    endif()

    # Verify the target is an executable
    get_target_property(TARGET_TYPE ${ARG_TARGET} TYPE)
    if(NOT TARGET_TYPE STREQUAL "EXECUTABLE")
        message(FATAL_ERROR "Target ${ARG_TARGET} must be an executable, but is ${TARGET_TYPE}")
    endif()

    get_target_property(TARGET_SOURCES ${ARG_TARGET} SOURCES)

    if(NOT TARGET_SOURCES OR TARGET_SOURCES STREQUAL "TARGET_SOURCES-NOTFOUND")
        return()
    endif()

    set(DISCOVERED_SUITES "")
    set(DISCOVERED_TESTS "")
    set(TEST_FILEPATHS "")

    foreach(SOURCE_FILE ${TARGET_SOURCES})
        get_filename_component(SOURCE_FILE_ABSOLUTE_PATH "${SOURCE_FILE}" ABSOLUTE)

        # Set CMake to reload when source changes to discover newly added tests.
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SOURCE_FILE_ABSOLUTE_PATH}")

        if(NOT EXISTS "${SOURCE_FILE_ABSOLUTE_PATH}")
            message(WARNING "Source file does not exist: ${SOURCE_FILE_ABSOLUTE_PATH}")
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

            # Check if the line contains a test registration (after comment removal)
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

    if(NOT DISCOVERED_TESTS)
        return()
    endif()

    # Create directory for generated main file
    set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_tests/${ARG_TARGET}")
    file(MAKE_DIRECTORY "${GENERATED_DIR}")

    # Generate main file with all tests and command-line filtering
    set(GENERATED_MAIN "${GENERATED_DIR}/run_tests.c")

    # Generate includes
    set(MAIN_CONTENT "/* Auto-generated test main for ${ARG_TARGET}*/\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}#include <string.h>\n\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}#include <microstrain_test/microstrain_test.h>\n\n")

    # Generate any required setup
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
    set(MAIN_CONTENT "${MAIN_CONTENT}    MICROSTRAIN_TEST_BEGIN();\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    \n")

    # Generate conditional execution logic for each test.
    list(LENGTH DISCOVERED_TESTS TEST_COUNT)
    math(EXPR LAST_INDEX "${TEST_COUNT} - 1")
    foreach(INDEX RANGE ${LAST_INDEX})
        # We need to set the filepath explicitly (for Unity) since it points to the generated
        # runner file instead.
        list(GET TEST_FILEPATHS ${INDEX} TEST_FILEPATH)
        list(GET DISCOVERED_SUITES ${INDEX} SUITE_NAME)
        list(GET DISCOVERED_TESTS ${INDEX} TEST_NAME)

        # Escape backslashes for C string literal
        string(REPLACE "\\" "\\\\" TEST_FILEPATH_ESCAPED "${TEST_FILEPATH}")

        set(MAIN_CONTENT "${MAIN_CONTENT}    if (!test_filter || strcmp(test_filter, \"${TEST_NAME}\") == 0) {\n")
        set(MAIN_CONTENT "${MAIN_CONTENT}        INTERNAL_RUN_MICROSTRAIN_TEST_CASE_AUTO_DISCOVER(${SUITE_NAME}, ${TEST_NAME}, \"${TEST_FILEPATH_ESCAPED}\");\n")
        set(MAIN_CONTENT "${MAIN_CONTENT}    }\n")
    endforeach()

    # Generate the end of the main function.
    set(MAIN_CONTENT "${MAIN_CONTENT}    \n")
    set(MAIN_CONTENT "${MAIN_CONTENT}    return MICROSTRAIN_TEST_END();\n")
    set(MAIN_CONTENT "${MAIN_CONTENT}}\n")

    # Write the generated contents to the main file.
    file(WRITE "${GENERATED_MAIN}" "${MAIN_CONTENT}")

    # Add generated main to existing executable
    message(STATUS "Adding generated main to executable ${ARG_TARGET}")
    target_sources(${ARG_TARGET} PRIVATE ${GENERATED_MAIN})

    # Register each test with CTest
    foreach(TEST_NAME ${DISCOVERED_TESTS})
        message(STATUS "  - Registering test: ${TEST_NAME}")
        # TODO: Add test suite to avoid naming conflicts
        add_test(NAME ${TEST_NAME} COMMAND ${ARG_TARGET} --test=${TEST_NAME})

        set_tests_properties(${TEST_NAME} PROPERTIES
            TIMEOUT 30
            LABELS "${TARGET_NAME}"
        )

        if(ARG_LABELS)
            set_tests_properties(${TEST_NAME} PROPERTIES LABELS ${ARG_LABELS})
        endif()

        if(ARG_DISABLED)
            set_tests_properties(${TEST_NAME} PROPERTIES DISABLED TRUE)
        endif()

        if(ARG_SEQUENTIAL)
            set_tests_properties(${TEST_NAME} PROPERTIES RESOURCE_LOCK ${ARG_TARGET})
        endif()
    endforeach()
endfunction()

# ---------------------------------------------------------------
# C++ discovery
# ---------------------------------------------------------------

function(microstrain_discover_tests_cpp)
    cmake_parse_arguments(
        ARG
        "SEQUENTIAL"
        "TARGET"
        "LABELS"
        ${ARGN}
    )

    if(MICROSTRAIN_TEST_USE_DOCTEST)
        include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

        if(ARG_SEQUENTIAL)
            doctest_discover_tests(
                ${ARG_TARGET}
                PROPERTIES
                    LABELS ${ARG_LABELS}
                    RESOURCE_LOCK ${ARG_TARGET}
            )
        else()
            doctest_discover_tests(
                ${ARG_TARGET}
                PROPERTIES
                    LABELS ${ARG_LABELS}
            )
        endif()
    endif()
endfunction()
