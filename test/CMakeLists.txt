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
    add_subdirectory("microstrain_test") # TODO: Remove when disbanded
    add_subdirectory("cpp")
endif()
