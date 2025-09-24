include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

add_executable(UnitTests
    "logging.unit.cpp"
    "strings.unit.cpp"
)

target_link_libraries(UnitTests
    ${MIP_LIBRARY}
    microstrain_test
)

doctest_discover_tests(UnitTests
    PROPERTIES LABELS "unit"
)

add_executable(IntegrationTests
    "mip.integration.cpp"
    # TODO: Broken, can't be fixed until naming conventions decided
    "packet_interface.integration.cpp"
)

target_link_libraries(IntegrationTests
    ${MIP_LIBRARY}
    microstrain_test
)

doctest_discover_tests(IntegrationTests
    PROPERTIES LABELS "integration"
)
