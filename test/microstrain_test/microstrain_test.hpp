/**
    TODO: This should be moved to its own repository so it can be used by multiple projects

    Wrapper library for C/C++ automated testing frameworks

    The goal is to provide a standardized interface for automated testing, regardless of the backend framework being
    used.
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
#include "doctest/doctest_wrappers.hpp"
#endif

/// Registers a test in the given test suite.
///
/// Tests are defined as: TEST(suite_name, test_name)
///
/// Test suites are useful for grouping related behaviors. The test suite will be created automatically if it doesn't
/// exist.
#ifndef TEST
#error "TEST is not implemented for the current backend")
#endif

