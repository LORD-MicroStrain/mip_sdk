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
/// This assertion is pseudo-safe if the expected string is passed as the second argument.
/// It will fail early if the actual string isn't null terminated at the same position as
/// the expected string.
///
/// It is reasonable to assume that the expected string is null terminated, since it is
/// almost always hard-coded by a test.
#define CSTR_EQ_IMPL(actual, expected, level)                                                                 \
    do                                                                                                        \
    {                                                                                                         \
        INFO("Actual:   " << std::string(actual));                                                            \
        INFO("Expected: " << std::string(expected));                                                          \
                                                                                                              \
        bool terminated = actual[strlen(expected)] == '\0';                                                   \
        if (!terminated)                                                                                      \
        {                                                                                                     \
            level##_MESSAGE(terminated, "The actual string is not terminated when the expected string is.");  \
            break;                                                                                            \
        }                                                                                                     \
                                                                                                              \
        level##_EQ(strcmp(actual, expected), 0);                                                              \
    } while(0)
