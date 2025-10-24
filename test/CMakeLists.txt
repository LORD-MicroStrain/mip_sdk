# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

include(FetchContent)

set(MICROSTRAIN_TEST_FRAMEWORK_C "unity" CACHE STRING "" FORCE)
set(MICROSTRAIN_TEST_FRAMEWORK_CPP "doctest" CACHE STRING "" FORCE)

FetchContent_Declare(microstrain_test
    GIT_REPOSITORY https://github.com/LORD-MicroStrain/microstrain_test.git
    GIT_TAG v0.2.5
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
