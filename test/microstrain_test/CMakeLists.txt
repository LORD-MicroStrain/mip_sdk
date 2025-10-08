# ---------------------------------------------------------------
# Framework backends
# ---------------------------------------------------------------

# C++
option(MICROSTRAIN_TEST_USE_DOCTEST "Use doctest as the backend framework" OFF)

# ---------------------------------------------------------------
# Library setup
# ---------------------------------------------------------------

add_library(microstrain_test STATIC)

target_include_directories(microstrain_test PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(microstrain_test PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/microstrain_test.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/doctest/doctest_wrappers.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/doctest/doctest_wrappers.cpp"
)

# ---------------------------------------------------------------
# Library compile settings
# ---------------------------------------------------------------

if(MSVC)
    set(COMPILE_OPTIONS
        "/W4" # Enable warnings
        "/WX" # Warnings as errors
        "/MP" # Multi-process compilation
    )
else()
    set(COMPILE_OPTIONS
        "-Wall"   # Enable warnings
        "-Wextra" # Enable extra warnings
        "-Werror" # Warnings as errors
    )
endif()

target_compile_options(microstrain_test PUBLIC
    "${COMPILE_OPTIONS}"
)

target_compile_definitions(microstrain_test PUBLIC
    "MICROSTRAIN_LOGGING_MAX_LEVEL=${MIP_LOGGING_MAX_LEVEL}"
)

# ---------------------------------------------------------------
# Framework setup
# ---------------------------------------------------------------

if(MICROSTRAIN_TEST_USE_DOCTEST)
    include(installation/InstallDoctest.cmake)

    target_compile_definitions(microstrain_test PUBLIC
        "MICROSTRAIN_TEST_USE_DOCTEST"
    )

    # TODO: Set up separate systems for parallel vs. sequential execution
    target_link_libraries(microstrain_test PUBLIC
        "doctest_with_main"
    )
endif()
