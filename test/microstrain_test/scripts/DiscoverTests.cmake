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
    # -------------------------------------------------

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
        message(WARNING "No sources found for target ${ARG_TARGET}, skipping test discovery")
        return()
    endif()

    set(DISCOVERED_TESTS "")

    foreach(SOURCE_FILE ${TARGET_SOURCES})
        get_filename_component(SOURCE_FILE_ABSOLUTE_PATH "${SOURCE_FILE}" ABSOLUTE)

        if(NOT EXISTS "${SOURCE_FILE_ABSOLUTE_PATH}")
            message(WARNING "Source file does not exist: ${SOURCE_FILE_ABSOLUTE_PATH}")
            continue()
        endif()

        file(READ "${SOURCE_FILE_ABSOLUTE_PATH}" FILE_CONTENT)

        # Remove block comments /* ... */
        # This regex handles multiline block comments.
        #
        # Reference:
        # ----------
        # 1)   /\\* ---> Match "/*"
        # 2) (...)* ---> Capture group
        #     2a)      [^*] ---> Any character that's not "*"
        #     2b)         | ---> OR
        #     2c) \\*+[^*/] ---> One or more "*" followed by something that's not "*" or "/"
        #     2d)         * ---> Repeat the whole group zero or more times
        # 3)  \\*+/ ---> Match one or more "*" followed by "/"
        #
        # This handles complex cases such as if asterisks are inside the comment, etc.
        string(REGEX REPLACE "/\\*([^*]|\\*+[^*/])*\\*+/" "" FILE_CONTENT "${FILE_CONTENT}")

        # Split file into lines to check for line comments.
        #
        # Replaces every ";" with "\\;" (so they aren't interpreted as the CMake list separator).
        string(REGEX REPLACE ";" "\\\\;" FILE_CONTENT "${FILE_CONTENT}")
        # Converts the file to a CMake list where each element is one line by replacing newlines
        # with the CMake list separator. This allows the lines to be looped over.
        string(REGEX REPLACE "\n" ";" FILE_LINES "${FILE_CONTENT}")

        foreach(LINE ${FILE_LINES})
            # Remove everything after // (line comments)
            #
            # Reference:
            # ----------
            # 1) // ---> Match "//"
            # 2) .* ---> Match any character zero or more times
            # 3)  $ ---> Match the end of the line (position after the last character)
            string(REGEX REPLACE "//.*$" "" LINE_WITHOUT_COMMENT "${LINE}")

            # Check if the line contains a test registration (after comment removal)
            #
            # Reference:
            # ----------
            # 1) Literal text to match
            # 2)   \\( ---> Match "("
            # 3) (...) ---> Capture group
            #    3a) [a-zA-Z0-9_]+ ---> Match one or more characters that are valid qualifiers
            # 4)   \\) ---> Match ")"
            string(REGEX MATCH "MICROSTRAIN_TEST_CASE\\(([a-zA-Z0-9_]+)\\)" MATCH "${LINE_WITHOUT_COMMENT}")
            if(MATCH)
                # Extracts the test name from the test registration.
                #
                # \\1 is the first capture group, which in this case is the argument passed to the test
                # registration call (the test name).
                string(REGEX REPLACE "MICROSTRAIN_TEST_CASE\\(([a-zA-Z0-9_]+)\\)" "\\1" TEST_NAME "${MATCH}")
                list(APPEND DISCOVERED_TESTS ${TEST_NAME})
            endif()
        endforeach()
    endforeach()

    if(DISCOVERED_TESTS)
        # Remove duplicate tests. Count the amount of tests before and after to see if any
        # were removed.
        list(LENGTH DISCOVERED_TESTS ORIGINAL_COUNT)
        list(REMOVE_DUPLICATES DISCOVERED_TESTS)
        list(LENGTH DISCOVERED_TESTS UNIQUE_COUNT)

        # Warn if duplicate tests were found
        math(EXPR DUPLICATE_COUNT "${ORIGINAL_COUNT} - ${UNIQUE_COUNT}")
        if(DUPLICATE_COUNT GREATER 0)
            message(WARNING "Found ${DUPLICATE_COUNT} duplicate test name(s) in target ${ARG_TARGET}. Each test will only be registered once.")
        endif()

        message(STATUS "Discovered ${UNIQUE_COUNT} test(s) in target ${ARG_TARGET}")
    else()
        message(STATUS "No tests discovered in target ${ARG_TARGET}")
        return()
    endif()

    # -------------------------------------------------
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
