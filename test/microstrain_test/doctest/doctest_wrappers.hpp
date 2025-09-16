#pragma once

#include <cstddef>

#include <doctest/doctest.h>

namespace detail
{
    enum class FAILURE_LEVEL
    {
        WARN,
        FAIL,
        EXIT
    };

    void cStringsEqual(const char* actual, const char* expected, FAILURE_LEVEL level);
    void charsEqual(char actual, char expected);
}

// Using Doctest's tagging system here so we can run specific test suites without
// having to use TEST_SUITE_BEGIN/TEST_SUITE_END.
#define TEST(suite_name, test_name) TEST_CASE("[" suite_name "] " test_name)

#define WARN_IF_NOT_TRUE(condition) WARN(condition)
#define FAIL_IF_NOT_TRUE(condition) CHECK(condition)
#define EXIT_IF_NOT_TRUE(condition) REQUIRE(condition)

#define WARN_IF_NOT_EQUAL(actual, expected) WARN_EQ(actual, expected)
#define FAIL_IF_NOT_EQUAL(actual, expected) CHECK_EQ(actual, expected)
#define EXIT_IF_NOT_EQUAL(actual, expected) REQUIRE_EQ(actual, expected)

#define WARN_IF_C_STRINGS_NOT_EQUAL(actual, expected) \
    detail::cStringsEqual(actual, expected, detail::FAILURE_LEVEL::WARN)
#define FAIL_IF_C_STRINGS_NOT_EQUAL(actual, expected) \
    detail::cStringsEqual(actual, expected, detail::FAILURE_LEVEL::FAIL)
#define EXIT_IF_C_STRINGS_NOT_EQUAL(actual, expected) \
    detail::cStringsEqual(actual, expected, detail::FAILURE_LEVEL::EXIT)

#define FAIL_IF_CHARS_NOT_EQUAL(actual, expected) detail::charsEqual(actual, expected)
