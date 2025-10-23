# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

set(MICROSTRAIN_TEST_USE_UNITY ON CACHE BOOL "" FORCE)
set(MICROSTRAIN_TEST_USE_DOCTEST ON CACHE BOOL "" FORCE)
# TODO: Will be moved to separate repo
add_subdirectory("microstrain_test")

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
