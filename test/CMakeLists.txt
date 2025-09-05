include(FetchContent)
FetchContent_Declare(doctest URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.12.zip)
FetchContent_MakeAvailable(doctest)

add_library(framework_wrappers INTERFACE)
target_include_directories(framework_wrappers INTERFACE "${CMAKE_CURRENT_LIST_DIR}")
target_compile_features(framework_wrappers INTERFACE cxx_std_17)

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
