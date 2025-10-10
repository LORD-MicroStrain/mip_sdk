/**
    TODO: This should be moved to its own repository so it can be used by multiple projects

    C++ framework API
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
    #include "doctest_wrappers.hpp"
#endif

/// @brief Registers a test in the given test suite.
#define TEST(suite_name, test_name) TEST_IMPLEMENTATION(suite_name, test_name)

