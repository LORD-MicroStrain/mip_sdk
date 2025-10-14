function(microstrain_discover_tests)
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
