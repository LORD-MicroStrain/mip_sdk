/**
    TODO: This should be moved to its own repository so it can be used by multiple projects

    TODO: Move documentation to markdown file(s)

    Wrapper library for C/C++ automated testing frameworks

    The goal is to provide a standardized interface for automated testing, regardless of the backend framework being
    used.
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
#include "doctest/doctest_wrappers.hpp"
#endif


/** Test registration */

/// Registers a test in the given test suite.
///
/// Tests are defined as: TEST(suite_name, test_name)
///
/// Test suites are useful for grouping related behaviors. The test suite will be created automatically if it doesn't
/// exist.
#ifndef TEST
#error "TEST is not implemented for the current backend"
#endif


/** Logging */

#ifndef LOG_ON_FAIL
#error "LOG_ON_FAIL is not implemented for the current backend."
#endif


/** Assertions
 *
 * There are three levels of failure:
 *   - WARN_* : Outputs a warning without failing the test.
 *   - FAIL_* : Fails the test but allows it to continue running.
 *   - FATAL_*: Fails and exits the test immediately.
 *
 *   The arguments for each call follow the order (actual, expected). The order of arguments matters, as some assertions
 *   make assumptions on the size or bounds of the actual argument based on the expected argument.
 *
 *   Ex: LEVEL_ASSERTION(actual, expected)
*/

/// -------------------------------------------------------------------------------------------- ///
/// The following are useful if you only care about whether a condition is true.
///
/// They might not display any values used in the expression for debugging.
/// -------------------------------------------------------------------------------------------- ///

#ifndef WARN_IF_NOT_TRUE
#error "WARN_IF_NOT_TRUE is not implemented for the current backend."
#endif

#ifndef FAIL_IF_NOT_TRUE
#error "FAIL_IF_NOT_TRUE is not implemented for the current backend."
#endif

#ifndef FATAL_IF_NOT_TRUE
#error "FATAL_IF_NOT_TRUE is not implemented for the current backend."
#endif

/// -------------------------------------------------------------------------------------------- ///
/// More specific checks for equality that print the values when debugging.
/// -------------------------------------------------------------------------------------------- ///

#ifndef WARN_IF_NOT_EQUAL
#error "WARN_IF_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FAIL_IF_NOT_EQUAL
#error "FAIL_IF_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FATAL_IF_NOT_EQUAL
#error "FATAL_IF_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef WARN_IF_EQUAL
#error "WARN_IF_EQUAL is not implemented for the current backend."
#endif

#ifndef FAIL_IF_EQUAL
#error "FAIL_IF_EQUAL is not implemented for the current backend."
#endif

#ifndef FATAL_IF_EQUAL
#error "FATAL_IF_EQUAL is not implemented for the current backend."
#endif

/// -------------------------------------------------------------------------------------------- ///
/// Same as the equality checks, but with additional safety and debugging info for C strings.
/// -------------------------------------------------------------------------------------------- ///

#ifndef WARN_IF_C_STRINGS_NOT_EQUAL
#error "WARN_IF_C_STRINGS_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FAIL_IF_C_STRINGS_NOT_EQUAL
#error "FAIL_IF_C_STRINGS_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FATAL_IF_C_STRINGS_NOT_EQUAL
#error "FATAL_IF_C_STRINGS_NOT_EQUAL is not implemented for the current backend."
#endif

/// -------------------------------------------------------------------------------------------- ///
/// For individual characters.
/// -------------------------------------------------------------------------------------------- ///

#ifndef WARN_IF_CHAR_NOT_EQUAL
#error "WARN_IF_CHAR_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FAIL_IF_CHAR_NOT_EQUAL
#error "FAIL_IF_CHAR_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FATAL_IF_CHAR_NOT_EQUAL
#error "FATAL_IF_CHAR_NOT_EQUAL is not implemented for the current backend."
#endif

/// -------------------------------------------------------------------------------------------- ///
/// These allow you to pass custom information to display if a test assertion fails.
/// -------------------------------------------------------------------------------------------- ///

#ifndef WARN_AND_LOG_IF_NOT_TRUE
#error "WARN_AND_LOG_IF_NOT_TRUE is not implemented for the current backend."
#endif

#ifndef FAIL_AND_LOG_IF_NOT_TRUE
#error "FAIL_AND_LOG_IF_NOT_TRUE is not implemented for the current backend."
#endif

#ifndef FATAL_AND_LOG_IF_NOT_TRUE
#error "FATAL_AND_LOG_IF_NOT_TRUE is not implemented for the current backend."
#endif

#ifndef WARN_AND_LOG_IF_NOT_EQUAL
#error "WARN_AND_LOG_IF_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FAIL_AND_LOG_IF_NOT_EQUAL
#error "FAIL_AND_LOG_IF_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FATAL_AND_LOG_IF_NOT_EQUAL
#error "FATAL_AND_LOG_IF_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef WARN_AND_LOG_IF_EQUAL
#error "WARN_AND_LOG_IF_EQUAL is not implemented for the current backend."
#endif

#ifndef FAIL_AND_LOG_IF_EQUAL
#error "FAIL_AND_LOG_IF_EQUAL is not implemented for the current backend."
#endif

#ifndef FATAL_AND_LOG_IF_EQUAL
#error "FATAL_AND_LOG_IF_EQUAL is not implemented for the current backend."
#endif

#ifndef WARN_AND_LOG_IF_C_STRINGS_NOT_EQUAL
#error "WARN_AND_LOG_IF_C_STRINGS_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FAIL_AND_LOG_IF_C_STRINGS_NOT_EQUAL
#error "FAIL_AND_LOG_IF_C_STRINGS_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FATAL_AND_LOG_IF_C_STRINGS_NOT_EQUAL
#error "FATAL_AND_LOG_IF_C_STRINGS_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef WARN_AND_LOG_IF_CHAR_NOT_EQUAL
#error "WARN_AND_LOG_IF_CHAR_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FAIL_AND_LOG_IF_CHAR_NOT_EQUAL
#error "FAIL_AND_LOG_IF_CHAR_NOT_EQUAL is not implemented for the current backend."
#endif

#ifndef FATAL_AND_LOG_IF_CHAR_NOT_EQUAL
#error "FATAL_AND_LOG_IF_CHAR_NOT_EQUAL is not implemented for the current backend."
#endif
