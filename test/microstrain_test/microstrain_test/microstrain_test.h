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

/// @brief Registers a unit test in the given test suite.
///
/// Individual unit tests are run in parallel from all other tests when CTest is set to run in parallel.
/// All unit tests can be run by passing `-L unit` to CTest.
#define UNIT_TEST(suite_name, test_name) \
    UNIT_TEST_IMPLEMENTATION(suite_name, test_name)

/// @brief Registers an integration test in the given test suite.
///
/// Integration tests run sequentially, even when CTest is set to run in parallel. They will run in parallel
/// from unit tests, however.
/// All integration tests can be run by passing `-L integration` to CTest.
#define INTEGRATION_TEST(suite_name, test_name) \
    INTEGRATION_TEST_IMPLEMENTATION(suite_name, test_name)

// -----------------------------------------------------------------------------------------------------------
// Test execution
// -----------------------------------------------------------------------------------------------------------

// TODO: This should eventually be removed by all test code and only be used by the test discovery system
/// @brief Runs a MicrostrainTest-defined test case.
#define RUN_MICROSTRAIN_TEST(suite_name, test_name) \
    RUN_MICROSTRAIN_TEST_IMPLEMENTATION(suite_name, test_name)
