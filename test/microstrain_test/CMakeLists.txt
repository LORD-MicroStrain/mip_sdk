option(MICROSTRAIN_TEST_USE_DOCTEST "Use doctest as the backend framework" ON)

if(MICROSTRAIN_TEST_USE_DOCTEST)
    include(FetchContent)
    FetchContent_Declare(doctest
        GIT_REPOSITORY "https://github.com/doctest/doctest.git"
        GIT_TAG "v2.4.12"
    )
    FetchContent_MakeAvailable(doctest)
endif()

set(MICROSTRAIN_TEST_LIBRARY "microstrain_test")

add_library("${MICROSTRAIN_TEST_LIBRARY}" STATIC)

target_include_directories("${MICROSTRAIN_TEST_LIBRARY}" PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources("${MICROSTRAIN_TEST_LIBRARY}" PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/microstrain_test.hpp"
)

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

target_compile_options(${MICROSTRAIN_TEST_LIBRARY} PUBLIC
    "${COMPILE_OPTIONS}"
)

target_compile_definitions("${MICROSTRAIN_TEST_LIBRARY}" PUBLIC
    "MICROSTRAIN_LOGGING_MAX_LEVEL=${MIP_LOGGING_MAX_LEVEL}"
)

if(MICROSTRAIN_TEST_USE_DOCTEST)
    target_sources("${MICROSTRAIN_TEST_LIBRARY}" PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/doctest/doctest_wrappers.hpp"
        "${CMAKE_CURRENT_LIST_DIR}/doctest/doctest_wrappers.cpp"
    )

    target_compile_definitions("${MICROSTRAIN_TEST_LIBRARY}" PUBLIC
        "MICROSTRAIN_TEST_USE_DOCTEST"
    )

    target_link_libraries("${MICROSTRAIN_TEST_LIBRARY}" PUBLIC
        "doctest_with_main"
    )
endif()
