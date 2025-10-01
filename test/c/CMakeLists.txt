
# ---------------------------------------------------------------
# Cmocka installation
# ---------------------------------------------------------------

# TODO: Move CMocka stuff to cmake script

include(FetchContent)

# Make sure these are always updated.
unset(WITH_EXAMPLES CACHE)
unset(UNIT_TESTING CACHE)
unset(BUILD_SHARED_LIBS CACHE)

# The defaults for these options aren't ideal, overriding them instead.
option(WITH_EXAMPLES "" OFF)
option(UNIT_TESTING "" OFF)
option(BUILD_SHARED_LIBS "" OFF) # CMocka builds shared by default but linking doesn't work on Windows

set(CMOCKA_PROJECT "cmocka")

FetchContent_Declare(
    "${CMOCKA_PROJECT}"
    URL "https://cmocka.org/files/1.1/cmocka-1.1.8.tar.xz"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable("${CMOCKA_PROJECT}")

# Include the CMocka CMake modules for easy integration
list(APPEND CMAKE_MODULE_PATH "${${CMOCKA_PROJECT}_SOURCE_DIR}/cmake/Modules")

include(AddCMockaTest)

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
    "cmocka"
)

target_link_libraries("${MIP_INTEGRATION_TEST_TARGET_C}" INTERFACE
    "${MIP_LIBRARIES}"
    "${MICROSTRAIN_LIBRARIES}"
    "cmocka"
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
