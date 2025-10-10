# ---------------------------------------------------------------
# Targets
# ---------------------------------------------------------------

set(MIP_UNIT_TEST_TARGET_C "${MIP_UNIT_TEST_TARGET}_c")
set(MIP_INTEGRATION_TEST_TARGET_C "${MIP_INTEGRATION_TEST_TARGET}_c")

add_library("${MIP_UNIT_TEST_TARGET_C}" INTERFACE)
add_library("${MIP_INTEGRATION_TEST_TARGET_C}" INTERFACE)

# ---------------------------------------------------------------
# Linking
# ---------------------------------------------------------------

target_link_libraries("${MIP_UNIT_TEST_TARGET_C}" INTERFACE
    "${MIP_LIBRARIES}"
    "${MICROSTRAIN_LIBRARIES}"
    MicrostrainTest::c
)

target_link_libraries("${MIP_INTEGRATION_TEST_TARGET_C}" INTERFACE
    "${MIP_LIBRARIES}"
    "${MICROSTRAIN_LIBRARIES}"
    MicrostrainTest::cpp
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
