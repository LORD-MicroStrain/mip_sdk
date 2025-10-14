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
///
#define MICROSTRAIN_TEST_CASE(suite_name, test_name) \
    MICROSTRAIN_TEST_CASE_IMPLEMENTATION(suite_name, test_name)

/// @brief Call this to begin the test registration process.
///
/// Don't use this directly if using automatic test discovery.
///
/// Register tests after this call. When done, call the END function.
///
#define MICROSTRAIN_TEST_BEGIN() \
    MICROSTRAIN_TEST_BEGIN_IMPLEMENTATION()

/// @brief Call this to end the test registration process.
///
/// Don't use this directly if using automatic test discovery.
///
/// No tests will be registered after this call. The result should be returned from the main function.
///
#define MICROSTRAIN_TEST_END() \
    MICROSTRAIN_TEST_END_IMPLEMENTATION()

// -----------------------------------------------------------------------------------------------------------
// Test execution
// -----------------------------------------------------------------------------------------------------------

/// @brief Runs a MicrostrainTest-defined test case.
///
/// Don't use this directly if using automatic test discovery.
///
#define RUN_MICROSTRAIN_TEST_CASE(suite_name, test_name) \
    RUN_MICROSTRAIN_TEST_CASE_IMPLEMENTATION(suite_name, test_name)
