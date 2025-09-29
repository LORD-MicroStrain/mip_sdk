#ifndef CMOCKA_WRAPPER_H
#define CMOCKA_WRAPPER_H

#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif // !_CRT_SECURE_NO_WARNINGS

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif // !WIN32_LEAN_AND_MEAN
#endif // _MSC_VER

// CMocka required headers
#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "cmocka.h"

typedef struct CMUnitTest CMUnitTest;

#define MICROSTRAIN_TEST_FAILURE_COUNT total_failures
#define MICROSTRAIN_TEST_INIT int MICROSTRAIN_TEST_FAILURE_COUNT = 0
#define MICROSTRAIN_TEST_SUITE_START(name) CMUnitTest* name = NULL; int name##_count = 0
#define MICROSTRAIN_TEST_SUITE_RUN(group_name, name) if (name##_count > 0) { MICROSTRAIN_TEST_FAILURE_COUNT += _cmocka_run_group_tests(group_name, name, name##_count, NULL, NULL); }
#define MICROSTRAIN_TEST_SUITE_END(name) free(name)
#define MICROSTRAIN_TEST_CASE(test) static void test(void** state)

#define MICROSTRAIN_TEST_ASSERT_TRUE(val, message) if (val) { print_message(message); } assert_true(val);
#define MICROSTRAIN_TEST_ASSERT_FALSE(val, message) if (!val) { print_message(message); } assert_false(val);

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------------------------------- */
/* Custom Assertions                                                                                 */
/* ------------------------------------------------------------------------------------------------- */

// Compares two characters and displays them (instead of their ASCII codes) when failed
#define assert_char_equal(a, b)        \
    do                                 \
    {                                  \
        if (a != b)                    \
        {                              \
            fail_msg("%c != \\0", a);  \
        }                              \
    } while (0)

/* ------------------------------------------------------------------------------------------------- */

// Helper function to check if a test name is in argv
static inline bool should_run_test(const char* testName)
{
    // If no specific tests are provided (only program name), run all tests
    if (__argc <= 1)
    {
        return true;
    }

    // Check if the test name is in the command line arguments
    for (int i = 1; i < __argc; ++i)
    {
        if (strcmp(__argv[i], testName) == 0)
        {
            return true;
        }
    }

    return false;
}

// Helper function to add a test to a dynamic array
static inline CMUnitTest* add_test(CMUnitTest* tests, int* count, CMUnitTest test)
{
    tests = (CMUnitTest*)realloc(tests, (*count + 1) * sizeof(CMUnitTest));

    if (tests == NULL)
    {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }

    tests[*count] = test;
    (*count)++;

    return tests;
}

#ifdef _MSC_VER
// MSVC doesn't work nicely with compound literals in C
static inline CMUnitTest microstrain_create_unit_case(const char* name, const CMUnitTestFunction test_function) \
{
    CMUnitTest microstrain_unit_test;

    microstrain_unit_test.name = name;
    microstrain_unit_test.test_func = test_function;
    microstrain_unit_test.setup_func = NULL;
    microstrain_unit_test.teardown_func = NULL;
    microstrain_unit_test.initial_state = NULL;

    return microstrain_unit_test;
}
#define microstrain_create_test(test) microstrain_create_unit_case(#test, test)
#else
#define microstrain_create_test cmocka_unit_test
#endif // _MSC_VER

#define MICROSTRAIN_TEST_ADD(test_suite, test) \
if (should_run_test(#test)) \
{ \
    test_suite = add_test(test_suite, &test_suite##_count, microstrain_create_test(test)); \
}

#endif // !CMOCKA_WRAPPER_H
