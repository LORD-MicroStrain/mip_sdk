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

    # Verify target exists
    if(NOT TARGET ${ARG_TARGET})
        message(FATAL_ERROR "Target ${ARG_TARGET} does not exist")
    endif()

    # Verify target is an executable
    get_target_property(TARGET_TYPE ${ARG_TARGET} TYPE)
    if(NOT TARGET_TYPE STREQUAL "EXECUTABLE")
        message(FATAL_ERROR "Target ${ARG_TARGET} must be an executable, but is ${TARGET_TYPE}")
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
