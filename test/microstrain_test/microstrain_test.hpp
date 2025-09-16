/**
    TODO: This should be moved to its own repository so it can be used by multiple projects

    TODO: Maybe move the documentation to a markdown file?

    Wrapper library for C/C++ automated testing frameworks

	The goal is to provide a standardized interface for automated testing, regardless of the backend framework being
	used.

	Test registration:

    - TEST(suite_name, test_name) - Registers a test in the given test suite. The test suite will be created
                                    automatically if it doesn't exist.

    Assertions:

    There are three levels of failure:
        - WARN_*: Doesn't fail the test but outputs a warning message
        - FAIL_*: Fails the test but allows it to finish.
        - EXIT_*: Fails the test and exits immediately

    The arguments for each call are of the order (actual, expected). The order of arguments matters,
    as some assertions make assumptions on the size or bounds of the actual argument based on the
    expected argument.

    Ex: ASSERTION(actual, expected)

    - *_IF_NOT_TRUE - Useful if you only care about whether a condition is true. Might not have full debugging info.
    - *_IF_NOT_EQUAL
    - *_IF_C_STRINGS_NOT_EQUAL - Same as equality check, but with additional safety and debugging output for C strings
*/
#pragma once

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
#include "doctest/doctest_wrappers.hpp"
#endif
