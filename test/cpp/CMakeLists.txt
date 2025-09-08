
add_executable(CppUnitTestRunner
    # TODO: Commented tests are disabled as they are either broken or integration tests
    # mip_parser_performance.cpp
    # test_mip.cpp
    # test_packet_interface.cpp
    # test_serial.cpp
    "logging.unit.cpp"
    "strings.unit.cpp"
)

target_link_libraries(CppUnitTestRunner
    ${MIP_LIBRARY}
    framework_wrappers
)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(CppUnitTestRunner)
