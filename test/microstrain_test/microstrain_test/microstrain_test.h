/**
    C framework API
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_UNITY
    #include "unity_wrappers.h"
#endif


// -----------------------------------------------------------------------------------------------------------
// Test registration
// -----------------------------------------------------------------------------------------------------------

/// @brief Registers a test case in the given test suite.
///
/// The test suite will be created if it doesn't already exist.
#define MICROSTRAIN_TEST_CASE(suite_name, test_name) \
    MICROSTRAIN_TEST_CASE_IMPLEMENTATION(suite_name, test_name)

// -----------------------------------------------------------------------------------------------------------
// Test execution
// -----------------------------------------------------------------------------------------------------------

// TODO: This should eventually be removed by all test code and only be used by the test discovery system
/// @brief Runs a MicrostrainTest-defined test case.
#define RUN_MICROSTRAIN_TEST(suite_name, test_name) \
    RUN_MICROSTRAIN_TEST_IMPLEMENTATION(suite_name, test_name)
