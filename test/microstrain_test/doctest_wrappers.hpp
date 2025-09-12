#pragma once

#include <cstddef>

#include <doctest/doctest.h>

// Using Doctest's tagging system here so we can run specific test suites without
// having to use TEST_SUITE_BEGIN/TEST_SUITE_END.
#define FRAMEWORK_ADD_TEST(suite_name, test_name) TEST_CASE("[" suite_name "] " test_name)

#define FRAMEWORK_WARN_TRUE(condition) WARN(condition)
#define FRAMEWORK_FAIL_TRUE(condition) CHECK(condition)
#define FRAMEWORK_EXIT_TRUE(condition) REQUIRE(condition)

#define FRAMEWORK_WARN_EQUAL(actual, expected) WARN_EQ(actual, expected)
#define FRAMEWORK_FAIL_EQUAL(actual, expected) CHECK_EQ(actual, expected)
#define FRAMEWORK_EXIT_EQUAL(actual, expected) REQUIRE_EQ(actual, expected)

namespace detail
{
}
