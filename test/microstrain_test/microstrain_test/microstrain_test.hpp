/**
    C++ framework API
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
    #include "doctest_wrappers.hpp"
#endif


// -----------------------------------------------------------------------------------------------------------
// Test registration
// -----------------------------------------------------------------------------------------------------------

/// @brief Registers a unit test in the given test suite.
///
/// Individual unit tests are run in parallel from all other tests when CTest is set to run in parallel.
/// All unit tests can be run by passing `-L unit` to CTest.
#define MICROSTRAIN_TEST_CASE(suite_name, test_name) \
    MICROSTRAIN_TEST_CASE_IMPLEMENTATION(suite_name, test_name)
