#pragma once

#include <cstddef>
#include <string>

#include <doctest/doctest.h>


// Using Doctest's tagging system here so we can run specific test suites without
// having to use TEST_SUITE_BEGIN/TEST_SUITE_END.
#define TEST(suite_name, test_name) \
    TEST_CASE("[" suite_name "] " test_name)

/// Compares two C strings for equality.
///
/// Not safe! Make sure both strings are null terminated first.
#define CHECK_CSTR_EQ(actual, expected)               \
    do                                                \
    {                                                 \
        INFO("Actual:   " << std::string(actual));    \
        INFO("Expected: " << std::string(expected));  \
        CHECK_EQ(strcmp(actual, expected), 0);        \
    } while(0)                                        \
