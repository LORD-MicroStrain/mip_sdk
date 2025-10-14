# ---------------------------------------------------------------
# Targets
# ---------------------------------------------------------------

set(MIP_UNIT_TEST_TARGET_CPP "${MIP_UNIT_TEST_TARGET}_cpp")
set(MIP_INTEGRATION_TEST_TARGET_CPP "${MIP_INTEGRATION_TEST_TARGET}_cpp")

add_executable("${MIP_UNIT_TEST_TARGET_CPP}")
add_executable("${MIP_INTEGRATION_TEST_TARGET_CPP}")

# ---------------------------------------------------------------
# Target sources
# ---------------------------------------------------------------

target_sources("${MIP_UNIT_TEST_TARGET_CPP}" PRIVATE
    "logging.unit.cpp"
    "strings.unit.cpp"
)

target_sources("${MIP_INTEGRATION_TEST_TARGET_CPP}" PRIVATE
    "mip.integration.cpp"
    # TODO: Test doesn't compile and needs to be fixed
    # "packet_interface.integration.cpp"
)

# ---------------------------------------------------------------
# Linking
# ---------------------------------------------------------------

target_link_libraries("${MIP_UNIT_TEST_TARGET_CPP}" PRIVATE
    "${MIP_LIBRARIES}"
    "${MICROSTRAIN_LIBRARIES}"
    microstrain_test::cpp
)

target_link_libraries("${MIP_INTEGRATION_TEST_TARGET_CPP}" PRIVATE
    "${MIP_LIBRARIES}"
    "${MICROSTRAIN_LIBRARIES}"
    microstrain_test::cpp
)

# ---------------------------------------------------------------
# Automatic test discovery
# ---------------------------------------------------------------

include(${microstrain_test_SOURCE_DIR}/scripts/DiscoverTests.cmake)

microstrain_discover_tests(
    TARGET "${MIP_UNIT_TEST_TARGET_CPP}"
    LABELS "unit"
)

microstrain_discover_tests(
    TARGET "${MIP_INTEGRATION_TEST_TARGET_CPP}"
    LABELS "integration"
    SEQUENTIAL
)
