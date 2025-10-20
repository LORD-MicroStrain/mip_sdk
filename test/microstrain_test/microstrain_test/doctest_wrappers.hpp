#pragma once

#include <string>

#include <doctest/doctest.h>

// -----------------------------------------------------------------------------------------------------------
// Test registration
// -----------------------------------------------------------------------------------------------------------

#define MICROSTRAIN_TEST_CASE_IMPL(suite_name, test_name) \
    TEST_CASE("[" suite_name "] " test_name)

// -----------------------------------------------------------------------------------------------------------
// C-string assertions
// -----------------------------------------------------------------------------------------------------------

// TODO: Can add wrapper macros for the doctest parts of this and make it framework-independent
//     ---> In this case, we could rename the file common.hpp or implementation.hpp
#define CSTR_EQ_IMPL(actual, expected, level)                                                       \
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

/// @brief Displays a warning, but does not fail the test when the given C strings are not equal.
///
/// This assertion is pseudo-safe if the expected string is passed as the second argument.
/// It will fail early if the actual string isn't null terminated at the same position as
/// the expected string.
///
/// It is reasonable to assume that the expected string is null terminated, since it is
/// almost always hard-coded by a test.
#define WARN_CSTR_EQ(actual, expected) CSTR_EQ_IMPL(actual, expected, WARN)

/// @brief Fails the test when the given C strings are not equal, but allows the test to keep running.
///
/// This assertion is pseudo-safe if the expected string is passed as the second argument.
/// It will fail early if the actual string isn't null terminated at the same position as
/// the expected string.
///
/// It is reasonable to assume that the expected string is null terminated, since it is
/// almost always hard-coded by a test.
#define CHECK_CSTR_EQ(actual, expected) CSTR_EQ_IMPL(actual, expected, CHECK)

/// @brief Fails and exits the test immediately when the given C strings are not equal.
///
/// This assertion is pseudo-safe if the expected string is passed as the second argument.
/// It will fail early if the actual string isn't null terminated at the same position as
/// the expected string.
///
/// It is reasonable to assume that the expected string is null terminated, since it is
/// almost always hard-coded by a test.
#define REQUIRE_CSTR_EQ(actual, expected) CSTR_EQ_IMPL(actual, expected, REQUIRE)

