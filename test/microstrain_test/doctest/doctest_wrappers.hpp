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
#define TEST(suite_name, test_name) TEST_CASE("[" suite_name "] " test_name)

#define WARN_IF_NOT_TRUE(condition) WARN(condition)
#define FAIL_IF_NOT_TRUE(condition) CHECK(condition)
#define EXIT_IF_NOT_TRUE(condition) REQUIRE(condition)

#define WARN_IF_NOT_EQUAL(actual, expected) WARN_EQ(actual, expected)
#define FAIL_IF_NOT_EQUAL(actual, expected) CHECK_EQ(actual, expected)
#define EXIT_IF_NOT_EQUAL(actual, expected) REQUIRE_EQ(actual, expected)

#define WARN_IF_C_STRINGS_ARE_NOT_EQUAL(actual, expected) \
    detail::verifyCStringsAreEqual(actual, expected, detail::FAILURE_LEVEL::WARN)
#define FAIL_IF_C_STRINGS_ARE_NOT_EQUAL(actual, expected) \
    detail::verifyCStringsAreEqual(actual, expected, detail::FAILURE_LEVEL::FAIL)
#define EXIT_IF_C_STRINGS_ARE_NOT_EQUAL(actual, expected) \
    detail::verifyCStringsAreEqual(actual, expected, detail::FAILURE_LEVEL::EXIT)

#define WARN_IF_BUFFER_IS_NOT_TERMINATED_AT_THE_END(buffer, contents_size) \
    detail::verifyBufferTerminated(buffer, contents_size, contents_size, detail::FAILURE_LEVEL::WARN)
#define FAIL_IF_BUFFER_IS_NOT_TERMINATED_AT_THE_END(buffer, contents_size) \
    detail::verifyBufferTerminated(buffer, contents_size, contents_size, detail::FAILURE_LEVEL::FAIL)
#define EXIT_IF_BUFFER_IS_NOT_TERMINATED_AT_THE_END(buffer, contents_size) \
    detail::verifyBufferTerminated(buffer, contents_size, contents_size, detail::FAILURE_LEVEL::EXIT)

#define WARN_IF_BUFFER_IS_NOT_TERMINATED_AT_POSITION(buffer, contents_size, position) \
    detail::verifyBufferTerminated(buffer, contents_size, position, detail::FAILURE_LEVEL::WARN)
#define FAIL_IF_BUFFER_IS_NOT_TERMINATED_AT_POSITION(buffer, contents_size, position) \
    detail::verifyBufferTerminated(buffer, contents_size, position, detail::FAILURE_LEVEL::FAIL)
#define EXIT_IF_BUFFER_IS_NOT_TERMINATED_AT_POSITION(buffer, contents_size, position) \
    detail::verifyBufferTerminated(buffer, contents_size, position, detail::FAILURE_LEVEL::EXIT)
