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

    void verifyCStringsAreEqual(const char* actual, const char* expected, FAILURE_LEVEL level);
    void verifyBufferTerminated(const char *buffer, size_t buffer_size, size_t position, FAILURE_LEVEL level);
}

// Using Doctest's tagging system here so we can run specific test suites without
// having to use TEST_SUITE_BEGIN/TEST_SUITE_END.
#define DETAIL_ADD_TEST(suite_name, test_name) TEST_CASE("[" suite_name "] " test_name)

#define DETAIL_WARN_TRUE(condition) WARN(condition)
#define DETAIL_FAIL_TRUE(condition) CHECK(condition)
#define DETAIL_EXIT_TRUE(condition) REQUIRE(condition)

#define DETAIL_WARN_EQUAL(actual, expected) WARN_EQ(actual, expected)
#define DETAIL_FAIL_EQUAL(actual, expected) CHECK_EQ(actual, expected)
#define DETAIL_EXIT_EQUAL(actual, expected) REQUIRE_EQ(actual, expected)

#define DETAIL_WARN_C_STRINGS_EQUAL(actual, expected) \
    detail::verifyCStringsAreEqual(actual, expected, detail::FAILURE_LEVEL::WARN)
#define DETAIL_FAIL_C_STRINGS_EQUAL(actual, expected) \
    detail::verifyCStringsAreEqual(actual, expected, detail::FAILURE_LEVEL::FAIL)
#define DETAIL_EXIT_C_STRINGS_EQUAL(actual, expected) \
    detail::verifyCStringsAreEqual(actual, expected, detail::FAILURE_LEVEL::EXIT)

#define DETAIL_WARN_BUFFER_TERMINATED(buffer, contents_size) \
    detail::verifyBufferTerminated(buffer, contents_size, contents_size, detail::FAILURE_LEVEL::WARN)
#define DETAIL_FAIL_BUFFER_TERMINATED(buffer, contents_size) \
    detail::verifyBufferTerminated(buffer, contents_size, contents_size, detail::FAILURE_LEVEL::FAIL)
#define DETAIL_EXIT_BUFFER_TERMINATED(buffer, contents_size) \
    detail::verifyBufferTerminated(buffer, contents_size, contents_size, detail::FAILURE_LEVEL::EXIT)

#define DETAIL_WARN_BUFFER_TERMINATED_AT_POSITION(buffer, contents_size, position) \
    detail::verifyBufferTerminated(buffer, contents_size, position, detail::FAILURE_LEVEL::WARN)
#define DETAIL_FAIL_BUFFER_TERMINATED_AT_POSITION(buffer, contents_size, position) \
    detail::verifyBufferTerminated(buffer, contents_size, position, detail::FAILURE_LEVEL::FAIL)
#define DETAIL_EXIT_BUFFER_TERMINATED_AT_POSITION(buffer, contents_size, position) \
    detail::verifyBufferTerminated(buffer, contents_size, position, detail::FAILURE_LEVEL::EXIT)
