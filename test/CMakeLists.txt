
set(TEST_DIR "${CMAKE_CURRENT_LIST_DIR}")

#
# MIP
#

set(TEST_MIP_SOURCES
    "${TEST_DIR}/mip/mip_packet_builder.c"
)

set(TEST_MIP_PARSER_SOURCES
)

add_executable(TestMipPacketBuilding "${TEST_DIR}/mip/test_mip_packet_builder.c")
add_executable(TestMipParsing        "${TEST_DIR}/mip/test_mip_parser.c")
add_executable(TestMipRandom         "${TEST_DIR}/mip/test_mip_random.c")

target_link_libraries(TestMipPacketBuilding mip)
target_link_libraries(TestMipParsing        mip)
target_link_libraries(TestMipRandom         mip)


add_test(NAME MipPacketBuilding COMMAND TestMipPacketBuilding)
add_test(NAME MipParsing        COMMAND TestMipParsing "mip_data.bin" WORKING_DIRECTORY "${TEST_DIR}/data")
add_test(NAME TestMipRandom     COMMAND TestMipRandom)
