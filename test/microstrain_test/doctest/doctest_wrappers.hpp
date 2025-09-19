#pragma once

#include <cstddef>

#include <doctest/doctest.h>

namespace detail
{
    enum class FailureLevel
    {
        WARN,
        FAIL,
        FATAL
    };

    void cStringsEqual(const char* actual, const char* expected, FailureLevel failure_level);
    void charEqual(char actual, char expected, FailureLevel failure_level);
}

// Using Doctest's tagging system here so we can run specific test suites without
// having to use TEST_SUITE_BEGIN/TEST_SUITE_END.
#define TEST(suite_name, test_name) TEST_CASE("[" suite_name "] " test_name)

#define WARN_IF_NOT_TRUE(condition) WARN(condition)
#define FAIL_IF_NOT_TRUE(condition) CHECK(condition)
#define FATAL_IF_NOT_TRUE(condition) REQUIRE(condition)

#define WARN_IF_NOT_EQUAL(actual, expected) WARN_EQ(actual, expected)
#define FAIL_IF_NOT_EQUAL(actual, expected) CHECK_EQ(actual, expected)
#define FATAL_IF_NOT_EQUAL(actual, expected) REQUIRE_EQ(actual, expected)

#define WARN_IF_C_STRINGS_NOT_EQUAL(actual, expected) detail::cStringsEqual(actual, expected, detail::FailureLevel::WARN)
#define FAIL_IF_C_STRINGS_NOT_EQUAL(actual, expected) detail::cStringsEqual(actual, expected, detail::FailureLevel::FAIL)
#define FATAL_IF_C_STRINGS_NOT_EQUAL(actual, expected) detail::cStringsEqual(actual, expected, detail::FailureLevel::FATAL)

#define WARN_IF_CHAR_NOT_EQUAL(actual, expected) detail::charEqual(actual, expected, detail::FailureLevel::WARN)
#define FAIL_IF_CHAR_NOT_EQUAL(actual, expected) detail::charEqual(actual, expected, detail::FailureLevel::FAIL)
#define FATAL_IF_CHAR_NOT_EQUAL(actual, expected) detail::charEqual(actual, expected, detail::FailureLevel::FATAL)
