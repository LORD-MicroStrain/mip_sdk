# ---------------------------------------------------------------
# Project setup
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)
project(microstrain_test)

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
)
target_sources(microstrain_test_cpp INTERFACE
    "microstrain_test/microstrain_test.hpp"
)

# Add namespace aliases
add_library(microstrain_test::c ALIAS microstrain_test_c)
add_library(microstrain_test::cpp ALIAS microstrain_test_cpp)

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

    target_link_libraries(microstrain_test_cpp INTERFACE
        doctest_with_main
    )
endif()
