/**
	Wrapper library for C/C++ automated testing frameworks

	The goal is to provide a standardized interface for testing, regardless of the
	backend framework being used.

	Currently, only EXPECT interfaces are implemented for custom assertions. They
    will be added as needed and when they can be fully tested.

	TODO: This should be moved to its own repository so it can be used by multiple
          projects

	TODO: Find a way to reuse custom logic between WARN, EXPECT, and ASSERT interfaces
	    ---> Maybe we can set up an enum to pass to a single function that then calls the
	         correct level of assertion?

    TODO: Split macro definitions into top-level header and implement functions in framework
          files
*/
#pragma once

// Allow the use of framework features that haven't been wrapped yet
#ifdef MICROSTRAIN_TEST_USE_DOCTEST
#include "doctest_wrappers.hpp"
#endif


namespace detail
{
    void check_c_strings_equal(const char* actual, const char* expected);

	void check_buffer_terminated(const char *buffer, size_t buffer_size, size_t position);
}

#define TEST(suite_name, test_name) FRAMEWORK_ADD_TEST(suite_name, test_name)

#define WARN_IF_NOT_TRUE(condition) FRAMEWORK_WARN_TRUE(condition)
#define FAIL_IF_NOT_TRUE(condition) FRAMEWORK_FAIL_TRUE(condition)
#define EXIT_IF_NOT_TRUE(condition) FRAMEWORK_EXIT_TRUE(condition)

#define WARN_IF_NOT_EQUAL(actual, expected) FRAMEWORK_WARN_EQUAL(actual, expected)
#define FAIL_IF_NOT_EQUAL(actual, expected) FRAMEWORK_FAIL_EQUAL(actual, expected)
#define EXIT_IF_NOT_EQUAL(actual, expected) FRAMEWORK_EXIT_EQUAL(actual, expected)

#define EXPECT_C_STRINGS_TO_BE_EQUAL(actual, expected) detail::check_c_strings_equal(actual, expected)

#define EXPECT_BUFFER_TO_BE_TERMINATED(buffer, buffer_size) detail::check_buffer_terminated(buffer, buffer_size, buffer_size);
#define EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(buffer, buffer_size, position) detail::check_buffer_terminated(buffer, buffer_size, position);