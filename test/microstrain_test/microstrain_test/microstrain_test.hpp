/**
    TODO: This should be moved to its own repository so it can be used by multiple projects

    C++ framework API
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
    #include "doctest_wrappers.hpp"
#endif


// -----------------------------------------------------------------------------------------------------------
// Test registration
// -----------------------------------------------------------------------------------------------------------

/// @brief Registers a test in the given test suite.
#define UNIT_TEST(suite_name, test_name) UNIT_TEST_IMPLEMENTATION(suite_name, test_name)

