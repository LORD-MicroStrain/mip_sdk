include(FetchContent)

# Make sure these are always updated.
unset(WITH_EXAMPLES CACHE)
unset(UNIT_TESTING CACHE)
unset(BUILD_SHARED_LIBS CACHE)

# The defaults for these options aren't ideal, overriding them instead.
option(WITH_EXAMPLES "" OFF)
option(UNIT_TESTING "" OFF)
# CMocka builds shared by default but linking doesn't work on Windows
option(BUILD_SHARED_LIBS "" OFF)

FetchContent_Declare(
    cmocka
    URL https://cmocka.org/files/1.1/cmocka-1.1.8.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(cmocka)

# Include the CMocka CMake modules for easy integration
list(APPEND CMAKE_MODULE_PATH "${cmocka_SOURCE_DIR}/cmake/Modules")

set(MIP_UNIT_TEST_TARGET_C "${MIP_UNIT_TEST_TARGET}_c")
set(MIP_INTEGRATION_TEST_TARGET_C "${MIP_INTEGRATION_TEST_TARGET}_c")

include(AddCMockaTest)

add_library("${MIP_UNIT_TEST_TARGET_C}" INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mip_cmocka.h
)

target_link_libraries("${MIP_UNIT_TEST_TARGET_C}" INTERFACE
    "${MIP_LIBRARIES}"
    "${MICROSTRAIN_LIBRARIES}"
    "cmocka"
)

target_include_directories("${MIP_UNIT_TEST_TARGET_C}" INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

add_subdirectory("microstrain")
#add_subdirectory("mip")
