set(TEST_DIR "${CMAKE_CURRENT_LIST_DIR}")

add_library(MicrostrainTest "${TEST_DIR}/test.h" "${TEST_DIR}/test.c")

#
# MIP
#

macro(add_mip_test name sources command)
    add_executable(${name} ${sources})

    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${MICROSTRAIN_SRC_DIR})
    target_link_libraries(${name} MicrostrainTest ${MIP_LIBRARY})

    add_test(${name} ${command} ${ARGN})
endmacro()

add_mip_test(TestMipPacketBuilding "${TEST_DIR}/c/mip/test_mip_packet_builder.c" TestMipPacketBuilding)
add_mip_test(TestMipParsing        "${TEST_DIR}/c/mip/test_mip_parser.c" TestMipParsing "${TEST_DIR}/data/mip_data.bin")
add_mip_test(TestMipRandom         "${TEST_DIR}/c/mip/test_mip_random.c" TestMipRandom)
add_mip_test(TestMipFields         "${TEST_DIR}/c/mip/test_mip_fields.c" TestMipFields)
add_mip_test(TestMipCpp            "${TEST_DIR}/cpp/mip/test_mip.cpp" TestMipCpp)
add_mip_test(TestMipPerf           "${TEST_DIR}/cpp/mip/mip_parser_performance.cpp" TestMipPerf)
add_mip_test(TestMipPacket         "${TEST_DIR}/cpp/mip/test_packet_interface.cpp" TestMipPacket)

#if(MICROSTRAIN_BUILD_EXAMPLES)
#    set(OUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/MipPacketExampleOutput.txt")
#    add_test(
#        NAME TestMipPacketExample
#        COMMAND diff -u "${CMAKE_CURRENT_LIST_DIR}/data/packet_example_cpp_check.txt" <(examples/MipPacketExample)
#    )
#endif()
