# TODO: Disabled tests currently don't compile.

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
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/../../c/microstrain/testutil_strings.c"
        "${CMAKE_CURRENT_LIST_DIR}/../../c/microstrain/testutil_strings.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/test_mip_formatting.cpp"
    COMMAND TestMipFormattingCpp
    USE_CPP TRUE
)

# TODO: Replace with valid check for IO support
target_compile_definitions(TestMipFormattingCpp PRIVATE
    "MICROSTRAIN_LOGGING_MAX_LEVEL=${MIP_LOGGING_MAX_LEVEL}"
)
