include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

add_executable(CppUnitTestRunner
    "logging.unit.cpp"
    "strings.unit.cpp"
)

target_link_libraries(CppUnitTestRunner
    ${MIP_LIBRARY}
    microstrain_test
)

doctest_discover_tests(CppUnitTestRunner)

add_executable(CppIntegrationTestRunner
    "mip.integration.cpp"
    "packet_interface.integration.cpp"
)

target_link_libraries(CppIntegrationTestRunner
    ${MIP_LIBRARY}
    microstrain_test
)

doctest_discover_tests(CppIntegrationTestRunner)
