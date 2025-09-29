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

#include <setjmp.h>

#include "cmocka.h"

typedef struct CMUnitTest CMUnitTest;

#define MICROSTRAIN_UNIT_TEST_FAILURES total_failures
#define MICROSTRAIN_UNIT_TESTS_INIT int MICROSTRAIN_UNIT_TEST_FAILURES = 0
#define MICROSTRAIN_TEST_SUITE_START(name) CMUnitTest* name = NULL; int name##_count = 0;
#define MICROSTRAIN_TEST_SUITE_RUN(group_name, name) if (name##_count > 0) { MICROSTRAIN_UNIT_TEST_FAILURES += _cmocka_run_group_tests(group_name, name, name##_count, NULL, NULL); }
#define MICROSTRAIN_TEST_SUITE_END(name) free(name); int name##_count = 0


#define MICROSTRAIN_ASSERT_TRUE(val, message) if (val) { print_message(message); } assert_true(val);
#define MICROSTRAIN_ASSERT_FALSE(val, message) if (!val) { print_message(message); } assert_false(val);

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
static inline CMUnitTest* add_test(CMUnitTest* tests, int* count, const CMUnitTest test)
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

// #define MICROSTRAIN_ADD_UNIT_TEST(test, test_suite, test_count) \
// if (should_run_test(#test)) \
// { \
//     test_suite = add_test(test_suite, test_count, cmocka_unit_test(test)); \
// }

// #define MICROSTRAIN_ADD_UNIT_TEST(test, test_suite, test_count) \
// do \
// { \
//     if (should_run_test(#test)) \
//     { \
//         test_suite = add_test(test_suite, &test_count, cmocka_unit_test(test)); \
//     } \
// } \
// while (0)
#define MICROSTRAIN_ADD_UNIT_TEST(test, test_suite, test_count) \
if (should_run_test(#test)) \
{ \
    test_suite = add_test(test_suite, &test_count, cmocka_unit_test(test)); \
}

#endif // !CMOCKA_WRAPPER_H
