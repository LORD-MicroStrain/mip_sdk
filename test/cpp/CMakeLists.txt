include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

add_executable(CppUnitTestRunner
    # TODO: Commented tests are disabled as they are either broken or integration tests
    # test_mip.cpp
    # test_packet_interface.cpp
    # test_serial.cpp
    "logging.unit.cpp"
    "strings.unit.cpp"
)
target_link_libraries(CppUnitTestRunner
    ${MIP_LIBRARY}
    microstrain_test
)
doctest_discover_tests(CppUnitTestRunner)

add_executable(CppIntegrationTestRunner
    # test_mip.cpp
    # test_packet_interface.cpp
    # test_serial.cpp
)
target_link_libraries(CppIntegrationTestRunner
    ${MIP_LIBRARY}
    microstrain_test
)
doctest_discover_tests(CppIntegrationTestRunner)
