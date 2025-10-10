# ---------------------------------------------------------------
# Framework backends
# ---------------------------------------------------------------

# C
option(MICROSTRAIN_TEST_USE_UNITY "Use unity as the backend framework for C" OFF)

# C++
option(MICROSTRAIN_TEST_USE_DOCTEST "Use doctest as the backend framework for C++" OFF)

# ---------------------------------------------------------------
# Library setup
# ---------------------------------------------------------------

add_library(microstrain_test INTERFACE)

target_include_directories(microstrain_test INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(microstrain_test INTERFACE
    "microstrain_test/microstrain_test.hpp"
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

target_compile_options(microstrain_test INTERFACE
    "${COMPILE_OPTIONS}"
)

target_compile_definitions(microstrain_test INTERFACE
    "MICROSTRAIN_LOGGING_MAX_LEVEL=${MIP_LOGGING_MAX_LEVEL}"
)


# ---------------------------------------------------------------
# Unity setup
# ---------------------------------------------------------------

if(MICROSTRAIN_TEST_USE_UNITY)
    include(scripts/InstallUnity.cmake)

    target_compile_definitions(microstrain_test INTERFACE
        "MICROSTRAIN_TEST_USE_UNITY"
    )

    target_link_libraries(microstrain_test INTERFACE
        unity
    )
endif()

# ---------------------------------------------------------------
# Doctest setup
# ---------------------------------------------------------------

if(MICROSTRAIN_TEST_USE_DOCTEST)
    include(scripts/InstallDoctest.cmake)

    target_compile_definitions(microstrain_test INTERFACE
        "MICROSTRAIN_TEST_USE_DOCTEST"
    )

    # TODO: Set up separate systems for parallel vs. sequential execution
    target_link_libraries(microstrain_test INTERFACE
        doctest_with_main
    )
endif()

# ---------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------

# Temporary function that adds a CTest test
function(add_unity_test)
    cmake_parse_arguments(
        ARG
        ""
        "NAME"
        "SOURCES;TARGET"
        ${ARGN}
    )
    add_executable(${ARG_NAME} ${ARG_SOURCES})
    target_link_libraries(${ARG_NAME} PRIVATE ${ARG_TARGET})
    add_test(NAME ${ARG_NAME} COMMAND ${ARG_NAME})
endfunction()
