# ---------------------------------------------------------------
# Targets
# ---------------------------------------------------------------

set(MIP_UNIT_TEST_TARGET_C "${MIP_UNIT_TEST_TARGET}_c")
set(MIP_INTEGRATION_TEST_TARGET_C "${MIP_INTEGRATION_TEST_TARGET}_c")

add_executable("${MIP_UNIT_TEST_TARGET_C}" INTERFACE)
add_executable("${MIP_INTEGRATION_TEST_TARGET_C}" INTERFACE)

# ---------------------------------------------------------------
# Linking
# ---------------------------------------------------------------

target_link_libraries("${MIP_UNIT_TEST_TARGET_C}" INTERFACE
    "${MIP_LIBRARIES}"
    "${MICROSTRAIN_LIBRARIES}"
    microstrain_test::c
)

target_link_libraries("${MIP_INTEGRATION_TEST_TARGET_C}" INTERFACE
    "${MIP_LIBRARIES}"
    "${MICROSTRAIN_LIBRARIES}"
    microstrain_test::cpp
)

# ---------------------------------------------------------------
# Compile definitions
# ---------------------------------------------------------------

target_compile_definitions("${MIP_UNIT_TEST_TARGET_C}" INTERFACE
    "MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOGGING_LEVEL_WARN_"
)

target_compile_definitions("${MIP_INTEGRATION_TEST_TARGET_C}" INTERFACE
    "MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOGGING_LEVEL_WARN_"
)

# ---------------------------------------------------------------
# Include directories
# ---------------------------------------------------------------

target_include_directories("${MIP_UNIT_TEST_TARGET_C}" INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_include_directories("${MIP_INTEGRATION_TEST_TARGET_C}" INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}"
)

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory("microstrain")
add_subdirectory("mip")

# ---------------------------------------------------------------
# Automatic test discovery
# ---------------------------------------------------------------

include(${microstrain_test_SOURCE_DIR}/scripts/DiscoverTests.cmake)

microstrain_discover_tests_c(
    TARGET "${MIP_UNIT_TEST_TARGET_C}"
    LABELS "unit"
)

microstrain_discover_tests_c(
    TARGET "${MIP_INTEGRATION_TEST_TARGET_C}"
    LABELS "integration"
    SEQUENTIAL
)
