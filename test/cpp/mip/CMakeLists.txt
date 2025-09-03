# TODO: Disabled tests are old and broken. Should we keep these?

#[[
add_mip_test(
    NAME TestMipPerf
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/mip_parser_performance.cpp"
    COMMAND TestMipPerf
    USE_CPP TRUE
)
]]

#[[
add_mip_test(
    NAME TestMipCpp
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_mip.cpp"
    COMMAND TestMipCpp
    USE_CPP TRUE
)
]]

#[[
add_mip_test(
    NAME TestMipPacket
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_packet_interface.cpp"
    COMMAND TestMipPacket
    USE_CPP TRUE
)
]]

add_mip_test(
    NAME TestMipFormattingCpp
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/../../c/microstrain/testutil_strings.c"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/../../c/microstrain/testutil_strings.h"
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_mip_formatting.cpp"
    COMMAND TestMipFormattingCpp
    USE_CPP TRUE
)
