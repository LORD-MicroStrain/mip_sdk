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

/// @brief Registers a test case in the given test suite.
///
/// The test suite will be created if it doesn't already exist.
#define MICROSTRAIN_TEST_CASE(suite_name, test_name) \
    MICROSTRAIN_TEST_CASE_IMPL(suite_name, test_name)
