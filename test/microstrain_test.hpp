/**
	Wrapper library for C/C++ automated testing frameworks

	The goal is to provide a standardized interface for testing, regardless of the
	backend framework being used.

	Currently, only EXPECT interfaces are implemented for custom assertions. They
    will be added as needed and when they can be fully tested.

	TODO: This should be moved to its own repository so it can be used by multiple
          projects

	TODO: Find a way to reuse custom logic between WARN, EXPECT, and ASSERT interfaces
*/
#pragma once

#ifdef USE_DOCTEST
#include <doctest/doctest.h>

namespace detail
{
    static constexpr size_t MAX_CHECK_STRING_LENGTH = 1024;

    void check_c_strings_equal(const char* string1, const char* string2);

	void check_buffer_terminated(const char *buffer, size_t position);
}

// Using Doctest's tagging system here so we can run specific test suites without
// having to use TEST_SUITE_BEGIN/TEST_SUITE_END.
#define TEST(suite_name, test_name) TEST_CASE("[" suite_name "] " test_name)

#define WARN_IF_NOT_TRUE(condition) WARN(condition)
#define EXPECT_TO_BE_TRUE(condition) CHECK(condition)
#define ASSERT_TO_BE_TRUE(condition) REQUIRE(condition)

#define WARN_IF_NOT_EQUAL(value1, value2) WARN_EQ(value1, value2)
#define EXPECT_TO_BE_EQUAL(value1, value2) CHECK_EQ(value1, value2)
#define ASSERT_TO_BE_EQUAL(value1, value2) REQUIRE_EQ(value1, value2)

#define EXPECT_C_STRINGS_TO_BE_EQUAL(value1, value2) detail::check_c_strings_equal(value1, value2)

#define EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(buffer, position) detail::check_buffer_terminated(buffer, position);
#endif