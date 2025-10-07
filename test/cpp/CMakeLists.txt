#[[
target_sources(UnitTests PRIVATE
    "logging.unit.cpp"
    "strings.unit.cpp"
)

target_sources(IntegrationTests PRIVATE
    "mip.integration.cpp"
    # TODO: Broken, can't be fixed until naming conventions decided
    #"packet_interface.integration.cpp"
)
]]

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
    microstrain_test
)

target_link_libraries("${MIP_INTEGRATION_TEST_TARGET_CPP}" PRIVATE
    "${MIP_LIBRARIES}"
    "${MICROSTRAIN_LIBRARIES}"
    microstrain_test
)

# ---------------------------------------------------------------
# Compile definitions
# ---------------------------------------------------------------

# TODO: Might not need these

target_compile_definitions("${MIP_UNIT_TEST_TARGET_CPP}" PRIVATE
    "MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOGGING_LEVEL_WARN_"
)

target_compile_definitions("${MIP_INTEGRATION_TEST_TARGET_CPP}" PRIVATE
    "MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOGGING_LEVEL_WARN_"
)

# ---------------------------------------------------------------
# Include directories
# ---------------------------------------------------------------

# TODO: Might not need these

target_include_directories("${MIP_UNIT_TEST_TARGET_CPP}" PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_include_directories("${MIP_INTEGRATION_TEST_TARGET_CPP}" PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
)

# ---------------------------------------------------------------
# Automatic test discovery
# ---------------------------------------------------------------

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

doctest_discover_tests("${MIP_UNIT_TEST_TARGET_CPP}"
    PROPERTIES
    LABELS "unit"
)

doctest_discover_tests("${MIP_INTEGRATION_TEST_TARGET_CPP}"
    PROPERTIES
    LABELS "integration"
)
