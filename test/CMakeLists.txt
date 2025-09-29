#[[
    Main CMake file for the test suite.

    Responsible for orchestrating test building and execution.
#]]

# TODO: Figure out why cmocka isn't working or switch to unity

# ---------------------------------------------------------------
# Executables
# ---------------------------------------------------------------

add_executable(UnitTests)
add_executable(IntegrationTests)

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory("c")

if(MICROSTRAIN_ENABLE_CPP)
    add_subdirectory("cpp")
endif()

add_subdirectory("microstrain_test")
add_subdirectory("cmocka")

# ---------------------------------------------------------------
# Library linking
# ---------------------------------------------------------------

target_link_libraries(UnitTests
    ${MIP_LIBRARY}
    microstrain_test
)

target_link_libraries(IntegrationTests
    ${MIP_LIBRARY}
    microstrain_test
)

# ---------------------------------------------------------------
# Automatic test discovery
# ---------------------------------------------------------------

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

doctest_discover_tests(UnitTests
    PROPERTIES
        LABELS "unit"
)

doctest_discover_tests(IntegrationTests
    PROPERTIES
        LABELS "integration"
)
