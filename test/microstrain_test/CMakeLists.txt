option(MICROSTRAIN_TEST_USE_DOCTEST "Use doctest as the backend framework" ON)

include(FetchContent)

if(MICROSTRAIN_TEST_USE_DOCTEST)
    FetchContent_Declare(doctest
        GIT_REPOSITORY "https://github.com/doctest/doctest.git"
        GIT_TAG "v2.4.12"
    )
    FetchContent_MakeAvailable(doctest)
endif()

# Framework wrapper library - Should probably be moved to its own repository at some point.
add_library(microstrain_test STATIC)
#[[
    "include/microstrain_test.hpp"
    "src/doctest_wrappers.hpp"
    "src/doctest_wrappers.cpp"
]]
#)
target_include_directories(microstrain_test
    PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_LIST_DIR}/src"
)

target_sources(microstrain_test
    PUBLIC "include/microstrain_test.hpp"
)

if(MICROSTRAIN_TEST_USE_DOCTEST)
    target_sources(microstrain_test
        PRIVATE
            "src/doctest_wrappers.hpp"
            "src/doctest_wrappers.cpp"
    )
    target_compile_definitions(microstrain_test
        PUBLIC MICROSTRAIN_TEST_USE_DOCTEST
    )
    target_link_libraries(microstrain_test
        PUBLIC doctest_with_main
    )
endif()
