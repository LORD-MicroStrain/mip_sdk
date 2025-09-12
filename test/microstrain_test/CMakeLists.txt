option(MICROSTRAIN_TEST_USE_DOCTEST "Use doctest as the backend framework" ON)

if(MICROSTRAIN_TEST_USE_DOCTEST)
    include(FetchContent)
    FetchContent_Declare(doctest
        GIT_REPOSITORY "https://github.com/doctest/doctest.git"
        GIT_TAG "v2.4.12"
    )
    FetchContent_MakeAvailable(doctest)
endif()

add_library(microstrain_test STATIC)

target_include_directories(microstrain_test
    PUBLIC "${CMAKE_CURRENT_LIST_DIR}"
    PRIVATE "${CMAKE_CURRENT_LIST_DIR}/src"
)

target_sources(microstrain_test
    PUBLIC "microstrain_test.hpp"
)

if(MICROSTRAIN_TEST_USE_DOCTEST)
    target_sources(microstrain_test
        PRIVATE
            "doctest/doctest_wrappers.hpp"
            "doctest/doctest_wrappers.cpp"
    )
    target_compile_definitions(microstrain_test
        PUBLIC MICROSTRAIN_TEST_USE_DOCTEST
    )
    target_link_libraries(microstrain_test
        PUBLIC doctest_with_main
    )
endif()
