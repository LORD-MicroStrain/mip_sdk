
set(TEST_DIR "${CMAKE_CURRENT_LIST_DIR}")

#
# MIP
#

set(TEST_MIP_SOURCES
    "${TEST_DIR}/mip/mip_packet_builder.c"
)

add_executable(TestMipPacketBuilding ${TEST_MIP_SOURCES})
target_link_libraries(TestMipPacketBuilding mip)

add_test(NAME MipPacketBuilding COMMAND TestMipPacketBuilding)
