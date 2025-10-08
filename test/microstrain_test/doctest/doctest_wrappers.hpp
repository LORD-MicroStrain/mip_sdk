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
}

// Using Doctest's tagging system here so we can run specific test suites without
// having to use TEST_SUITE_BEGIN/TEST_SUITE_END.
#define TEST(suite_name, test_name) \
    TEST_CASE("[" suite_name "] " test_name)

/* ---------------------------------------------------------------------------------------------- */

#define LOG_ON_FAIL(message) \
    INFO(message)

/* ---------------------------------------------------------------------------------------------- */


#define WARN_IF_C_STRINGS_NOT_EQUAL(actual, expected) \
    detail::cStringsEqual(actual, expected, detail::FailureLevel::WARN)

#define FAIL_IF_C_STRINGS_NOT_EQUAL(actual, expected) \
    detail::cStringsEqual(actual, expected, detail::FailureLevel::FAIL)

#define FATAL_IF_C_STRINGS_NOT_EQUAL(actual, expected) \
    detail::cStringsEqual(actual, expected, detail::FailureLevel::FATAL)

/* ---------------------------------------------------------------------------------------------- */

#define WARN_AND_LOG_IF_C_STRINGS_NOT_EQUAL(actual, expected) \
    do { INFO(message); detail::cStringsEqual(actual, expected, detail::FailureLevel::WARN); } while (false)

#define FAIL_AND_LOG_IF_C_STRINGS_NOT_EQUAL(actual, expected) \
    do { INFO(message); detail::cStringsEqual(actual, expected, detail::FailureLevel::FAIL); } while (false)

#define FATAL_AND_LOG_IF_C_STRINGS_NOT_EQUAL(actual, expected) \
    do { INFO(message); detail::cStringsEqual(actual, expected, detail::FailureLevel::FATAL); } while (false)
