
add_executable(CppUnitTestRunner
    # TODO: Commented tests are disable as they are either broken or integration tests
    # mip_parser_performance.cpp
    # test_mip.cpp
    # test_packet_interface.cpp
    # test_serial.cpp
    # TODO: Convert to new format
    "test_mip_formatting.cpp"
    #"test_strings.cpp"
)

target_link_libraries(CppUnitTestRunner
    ${MIP_LIBRARY}
    doctest_with_main
)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(CppUnitTestRunner)
