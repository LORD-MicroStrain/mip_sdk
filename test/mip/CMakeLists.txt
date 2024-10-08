#
# MIP base library
#

macro(add_mip_test name sources command)
    add_executable(${name} ${sources})

    target_include_directories(${name} PRIVATE ${MICROSTRAIN_SRC_DIR})
    target_link_libraries(${name} ${MIP_LIBRARY})

    add_test(${name} ${command} ${ARGN})
endmacro()

add_mip_test(TestMipPacketBuilding "${CMAKE_CURRENT_LIST_DIR}/test_mip_packet_builder.c" TestMipPacketBuilding)
add_mip_test(TestMipParsing        "${CMAKE_CURRENT_LIST_DIR}/test_mip_parser.c" TestMipParsing "${CMAKE_CURRENT_LIST_DIR}/../data/mip_data.bin")
add_mip_test(TestMipRandom         "${CMAKE_CURRENT_LIST_DIR}/test_mip_random.c" TestMipRandom)
add_mip_test(TestMipFields         "${CMAKE_CURRENT_LIST_DIR}/test_mip_fields.c" TestMipFields)
add_mip_test(TestMipCpp            "${CMAKE_CURRENT_LIST_DIR}/test_mip.cpp" TestMipCpp)
add_mip_test(TestMipPerf           "${CMAKE_CURRENT_LIST_DIR}/mip_parser_performance.cpp" TestMipPerf)
