/**
    C++ framework API
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
    #include "doctest_wrappers.hpp"
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
/// @param suite_name A string, which may contain any characters.
/// @param test_name A string, which may contain any characters.
///
#define MICROSTRAIN_TEST_CASE(suite_name, test_name) \
    MICROSTRAIN_TEST_CASE_IMPL(suite_name, test_name)
