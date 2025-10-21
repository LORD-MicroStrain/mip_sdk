/**
    C framework API
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_UNITY
    #include "unity_wrappers.h"
#endif

// -----------------------------------------------------------------------------------------------------------
// Automatic test registration and discovery
// -----------------------------------------------------------------------------------------------------------

/// @brief Registers a test case in the given test suite.
///
/// All tests in a test suite can be run together by passing the suite name as
/// a label to CTest. This currently only works with automatic test discovery,
/// or if you set the label for a CTest to the suite name manually.
///
/// @param suite_name Currently needs to be a valid function identifier.
/// @param test_name Currently needs to be a valid function identifier.
///
#define MICROSTRAIN_TEST_CASE(suite_name, test_name) \
    MICROSTRAIN_TEST_CASE_IMPL(suite_name, test_name)

// -----------------------------------------------------------------------------------------------------------
// Manual test registration and discovery
// -----------------------------------------------------------------------------------------------------------

/// @brief Completes any required setup before tests can be run.
///
/// DON'T use this directly if using automatic test discovery. This should be called
/// near the top of the main file, before the main function definition.
///
#define MICROSTRAIN_TEST_DEFAULT_SETUP() \
    MICROSTRAIN_TEST_DEFAULT_SETUP_IMPL()

/// @brief Begins the test registration process.
///
/// DON'T use this directly if using automatic test discovery. If manually discovering
/// tests, register tests after this call in the main function. When done, call the
/// END macro.
///
#define MICROSTRAIN_TEST_BEGIN() \
    MICROSTRAIN_TEST_BEGIN_IMPL()

/// @brief Ends the test registration process.
///
/// DON'T use this directly if using automatic test discovery. If manually discovering
/// tests, no tests will be registered after this call in the main function. The result
/// should be returned from the main function.
///
#define MICROSTRAIN_TEST_END() \
    MICROSTRAIN_TEST_END_IMPL()

/// @brief Runs a MicrostrainTest-defined test case.
///
/// DON'T use this directly if using automatic test discovery. Call this in the main
/// function for each test to run.
///
/// @param suite_name Currently needs to be a valid function identifier.
/// @param test_name Currently needs to be a valid function identifier.
///
#define RUN_MICROSTRAIN_TEST_CASE(suite_name, test_name) \
    RUN_MICROSTRAIN_TEST_CASE_IMPL(suite_name, test_name)

// -----------------------------------------------------------------------------------------------------------
// Internals
// -----------------------------------------------------------------------------------------------------------

/// @brief DON'T use directly! This is a modified version to work with automatic test discovery.
#define INTERNAL_RUN_MICROSTRAIN_TEST_CASE_AUTO_DISCOVER(suite_name, test_name, file_path) \
    INTERNAL_RUN_MICROSTRAIN_TEST_CASE_AUTO_DISCOVER_IMPL(suite_name, test_name, file_path)
