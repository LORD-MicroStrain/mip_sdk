/**
    TODO: This should be moved to its own repository so it can be used by multiple projects

    Library for C/C++ automated testing that provides common wrappers for the following frameworks:
        * Doctest
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
    #include "doctest/doctest_wrappers.hpp"

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

    /// @brief Registers a test in the given test suite.
    #define TEST(suite_name, test_name) TEST_CASE("[" suite_name "] " test_name)
#endif

