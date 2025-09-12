/**
	Wrapper library for C/C++ automated testing frameworks

	The goal is to provide a standardized interface for testing, regardless of the
	backend framework being used.

	Assertions are invoked like this: FAILURE_LEVEL_ASSERTION.

	There are three levels of failure:
	    * WARN: Doesn't fail the test but outputs a warning message
	    * FAIL: Fails the test but allows it to finish.
	    * EXIT: Fails the test and exits immediately

	TODO: This should be moved to its own repository so it can be used by multiple
          projects
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
#include "../src/doctest_wrappers.hpp"
#endif


// ------ Test registration --------------------------------------------------------------------- //

/// Registers a standard test in the given test suite.
///
/// Test suites are useful for grouping related behaviors. The test specified test suite will be
/// created automatically if it doesn't exist. Tests can then be run altogether, by test suite,
/// individually by test name, etc.
#define TEST(suite_name, test_name) FRAMEWORK_ADD_TEST(suite_name, test_name)


// ------ Assertions ---------------------------------------------------------------------------- //

/// The specific checks may provide more debugging information, but these can be helpful if you only
/// care about whether a condition is true.
#define WARN_IF_NOT_TRUE(condition) FRAMEWORK_WARN_TRUE(condition)
#define FAIL_IF_NOT_TRUE(condition) FRAMEWORK_FAIL_TRUE(condition)
#define EXIT_IF_NOT_TRUE(condition) FRAMEWORK_EXIT_TRUE(condition)

#define WARN_IF_NOT_EQUAL(actual, expected) FRAMEWORK_WARN_EQUAL(actual, expected)
#define FAIL_IF_NOT_EQUAL(actual, expected) FRAMEWORK_FAIL_EQUAL(actual, expected)
#define EXIT_IF_NOT_EQUAL(actual, expected) FRAMEWORK_EXIT_EQUAL(actual, expected)

/// Safe C string checks providing full debugging output, including the values of the strings.
/// The order of arguments matters, as the expected string is the only one that can be safely assumed
/// to be zero terminated (since it's defined by the test).
#define WARN_IF_C_STRINGS_ARE_NOT_EQUAL(actual, expected) FRAMEWORK_WARN_C_STRINGS_EQUAL(actual, expected)
#define FAIL_IF_C_STRINGS_ARE_NOT_EQUAL(actual, expected) FRAMEWORK_FAIL_C_STRINGS_EQUAL(actual, expected)
#define EXIT_IF_C_STRINGS_ARE_NOT_EQUAL(actual, expected) FRAMEWORK_EXIT_C_STRINGS_EQUAL(actual, expected)

/// Pass the size of the string contents, not the maximum size of the buffer.
#define WARN_IF_BUFFER_IS_NOT_TERMINATED_AT_THE_END(buffer, contents_size) FRAMEWORK_WARN_BUFFER_TERMINATED(buffer, contents_size)
#define FAIL_IF_BUFFER_IS_NOT_TERMINATED_AT_THE_END(buffer, contents_size) FRAMEWORK_FAIL_BUFFER_TERMINATED(buffer, contents_size)
#define EXIT_IF_BUFFER_IS_NOT_TERMINATED_AT_THE_END(buffer, contents_size) FRAMEWORK_EXIT_BUFFER_TERMINATED(buffer, contents_size)

#define WARN_IF_BUFFER_IS_NOT_TERMINATED_AT_POSITION(buffer, contents_size, position) FRAMEWORK_WARN_BUFFER_TERMINATED_AT_POSITION(buffer, contents_size, position)
#define FAIL_IF_BUFFER_IS_NOT_TERMINATED_AT_POSITION(buffer, contents_size, position) FRAMEWORK_FAIL_BUFFER_TERMINATED_AT_POSITION(buffer, contents_size, position)
#define EXIT_IF_BUFFER_IS_NOT_TERMINATED_AT_POSITION(buffer, contents_size, position) FRAMEWORK_EXIT_BUFFER_TERMINATED_AT_POSITION(buffer, contents_size, position)
