# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

include(FetchContent)

set(MICROSTRAIN_TEST_USE_UNITY ON CACHE BOOL "" FORCE)
set(MICROSTRAIN_TEST_USE_DOCTEST ON CACHE BOOL "" FORCE)

FetchContent_Declare(microstrain_test
    GIT_REPOSITORY https://github.com/LORD-MicroStrain/microstrain_test.git
    GIT_TAG v0.1.0
)

FetchContent_MakeAvailable(microstrain_test)

# ---------------------------------------------------------------
# Target templates
# ---------------------------------------------------------------

# See the C and CPP subdirectories for what these are expanded to.

set(MIP_UNIT_TEST_TARGET "mip_unit_tests")
set(MIP_INTEGRATION_TEST_TARGET "mip_integration_tests")

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory("c")

if(MICROSTRAIN_ENABLE_CPP)
    add_subdirectory("cpp")
endif()
