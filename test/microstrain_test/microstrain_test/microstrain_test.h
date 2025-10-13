/**
    TODO: This should be moved to its own repository so it can be used by multiple projects

    C framework API
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_UNITY
    #include "unity_wrappers.h"
#endif


// -----------------------------------------------------------------------------------------------------------
// Test registration
// -----------------------------------------------------------------------------------------------------------

/// @brief Registers a test in the given test suite.
#define UNIT_TEST(suite_name, test_name) \
    UNIT_TEST_IMPLEMENTATION(suite_name, test_name)

// -----------------------------------------------------------------------------------------------------------
// Test execution
// -----------------------------------------------------------------------------------------------------------

// TODO: This should eventually be removed by all test code and only be used by the test discovery system
/// @brief Runs a MicrostrainTest-defined test case.
#define RUN_MICROSTRAIN_TEST(suite_name, test_name) \
    RUN_MICROSTRAIN_TEST_IMPLEMENTATION(suite_name, test_name)
