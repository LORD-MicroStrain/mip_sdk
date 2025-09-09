
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
    PUBLIC USE_DOCTEST # Swap for other framework if needed.
)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/c")
if(MICROSTRAIN_ENABLE_CPP)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/cpp")
endif()

# TODO: Can we get rid of these?
#set(SINGLE_VALUES NAME COMMAND)
#set(MULTI_VALUES SOURCES ADDL_LINK_LIBRARIES ARGS)
#set(MIP_TEST_DATA_BIN "${CMAKE_CURRENT_LIST_DIR}/data/mip_data.bin")
#if(MICROSTRAIN_BUILD_EXAMPLES)
#    set(OUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/MipPacketExampleOutput.txt")
#    add_test(
#        NAME TestMipPacketExample
#        COMMAND diff -u "${CMAKE_CURRENT_LIST_DIR}/data/packet_example_cpp_check.txt" <(examples/MipPacketExample)
#    )
#endif()
