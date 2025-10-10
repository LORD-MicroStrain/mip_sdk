# ---------------------------------------------------------------
# Project setup
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 3.14)
project(MicrostrainTest)

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

add_library(microstrain_test_c INTERFACE)
add_library(microstrain_test_cpp INTERFACE)

set_target_properties(microstrain_test_c PROPERTIES
    LINKER_LANGUAGE C
)
set_target_properties(microstrain_test_cpp PROPERTIES
    LINKER_LANGUAGE CXX
)

target_include_directories(microstrain_test_c INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}"
)
target_include_directories(microstrain_test_cpp INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(microstrain_test_c INTERFACE
    "microstrain_test/microstrain_test.h"
    "microstrain_test/unity_wrappers.h" # TODO: Remove
)
target_sources(microstrain_test_cpp INTERFACE
    "microstrain_test/microstrain_test.hpp"
)

# Add namespace aliases
add_library(MicrostrainTest::c ALIAS microstrain_test_c)
add_library(MicrostrainTest::cpp ALIAS microstrain_test_cpp)

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

target_compile_options(microstrain_test_c INTERFACE
    "${COMPILE_OPTIONS}"
)
target_compile_options(microstrain_test_cpp INTERFACE
    "${COMPILE_OPTIONS}"
)

target_compile_definitions(microstrain_test_c INTERFACE
    "MICROSTRAIN_LOGGING_MAX_LEVEL=${MIP_LOGGING_MAX_LEVEL}"
)
target_compile_definitions(microstrain_test_cpp INTERFACE
    "MICROSTRAIN_LOGGING_MAX_LEVEL=${MIP_LOGGING_MAX_LEVEL}"
)

# ---------------------------------------------------------------
# (C) Unity setup
# ---------------------------------------------------------------

if(MICROSTRAIN_TEST_USE_UNITY)
    include(scripts/InstallUnity.cmake)

    target_compile_definitions(microstrain_test_c INTERFACE
        "MICROSTRAIN_TEST_USE_UNITY"
    )

    target_link_libraries(microstrain_test_c INTERFACE
        unity
    )
endif()

# ---------------------------------------------------------------
# (C++) Doctest setup
# ---------------------------------------------------------------

if(MICROSTRAIN_TEST_USE_DOCTEST)
    include(scripts/InstallDoctest.cmake)

    target_compile_definitions(microstrain_test_cpp INTERFACE
        "MICROSTRAIN_TEST_USE_DOCTEST"
    )

    # TODO: Set up separate systems for parallel vs. sequential execution
    target_link_libraries(microstrain_test_cpp INTERFACE
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
        "DISABLED"
        "NAME;TARGET"
        "SOURCES;LABELS"
        ${ARGN}
    )

    add_executable(${ARG_NAME} ${ARG_SOURCES})
    target_link_libraries(${ARG_NAME} PRIVATE ${ARG_TARGET})
    add_test(NAME ${ARG_NAME} COMMAND ${ARG_NAME})

    if(ARG_LABELS)
        set_tests_properties(${ARG_NAME} PROPERTIES LABELS ${ARG_LABELS})
    endif()

    if(ARG_DISABLED)
        set_tests_properties(${ARG_NAME} PROPERTIES DISABLED TRUE)
    endif()
endfunction()
