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

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
#include "doctest_wrappers.hpp"
#endif

#define TEST(suite_name, test_name) FRAMEWORK_ADD_TEST(suite_name, test_name)

#define WARN_IF_NOT_TRUE(condition) FRAMEWORK_WARN_TRUE(condition)
#define FAIL_IF_NOT_TRUE(condition) FRAMEWORK_FAIL_TRUE(condition)
#define EXIT_IF_NOT_TRUE(condition) FRAMEWORK_EXIT_TRUE(condition)

#define WARN_IF_NOT_EQUAL(actual, expected) FRAMEWORK_WARN_EQUAL(actual, expected)
#define FAIL_IF_NOT_EQUAL(actual, expected) FRAMEWORK_FAIL_EQUAL(actual, expected)
#define EXIT_IF_NOT_EQUAL(actual, expected) FRAMEWORK_EXIT_EQUAL(actual, expected)

#define WARN_IF_C_STRINGS_ARE_NOT_EQUAL(actual, expected) FRAMEWORK_WARN_C_STRINGS_EQUAL(actual, expected)
#define FAIL_IF_C_STRINGS_ARE_NOT_EQUAL(actual, expected) FRAMEWORK_FAIL_C_STRINGS_EQUAL(actual, expected)
#define EXIT_IF_C_STRINGS_ARE_NOT_EQUAL(actual, expected) FRAMEWORK_EXIT_C_STRINGS_EQUAL(actual, expected)

#define WARN_IF_BUFFER_IS_NOT_TERMINATED_AT_THE_END(buffer, buffer_size) FRAMEWORK_WARN_BUFFER_TERMINATED(buffer, buffer_size)
#define FAIL_IF_BUFFER_IS_NOT_TERMINATED_AT_THE_END(buffer, buffer_size) FRAMEWORK_FAIL_BUFFER_TERMINATED(buffer, buffer_size)
#define EXIT_IF_BUFFER_IS_NOT_TERMINATED_AT_THE_END(buffer, buffer_size) FRAMEWORK_EXIT_BUFFER_TERMINATED(buffer, buffer_size)

#define WARN_IF_BUFFER_IS_NOT_TERMINATED_AT_POSITION(buffer, buffer_size, position) FRAMEWORK_WARN_BUFFER_TERMINATED_AT_POSITION(buffer, buffer_size, position)
#define FAIL_IF_BUFFER_IS_NOT_TERMINATED_AT_POSITION(buffer, buffer_size, position) FRAMEWORK_FAIL_BUFFER_TERMINATED_AT_POSITION(buffer, buffer_size, position)
#define EXIT_IF_BUFFER_IS_NOT_TERMINATED_AT_POSITION(buffer, buffer_size, position) FRAMEWORK_EXIT_BUFFER_TERMINATED_AT_POSITION(buffer, buffer_size, position)
