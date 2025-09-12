# TODO: Add check for use doctest option
include(FetchContent)
FetchContent_Declare(doctest
    GIT_REPOSITORY "https://github.com/doctest/doctest.git"
    GIT_TAG "v2.4.12"
)
FetchContent_MakeAvailable(doctest)

# Framework wrapper library - Should probably be moved to its own repository at some point.
add_library(microstrain_test
    "microstrain_test.hpp"
    "microstrain_test.cpp"
)
target_include_directories(microstrain_test
    PUBLIC "${CMAKE_CURRENT_LIST_DIR}"
)
target_link_libraries(microstrain_test
    PUBLIC doctest_with_main
)
target_compile_definitions(microstrain_test
    PUBLIC MICROSTRAIN_TEST_USE_DOCTEST # Swap for other framework if needed.
)
