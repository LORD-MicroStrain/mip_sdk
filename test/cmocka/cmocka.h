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

#include <cmocka.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Helper function to check if a test name is in argv
inline bool should_run_test(const int argc, char* argv[], const char* testName)
{
    // If no specific tests are provided (only program name), run all tests
    if (argc <= 1)
    {
        return true;
    }

    // Check if the test name is in the command line arguments
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], testName) == 0)
        {
            return true;
        }
    }

    return false;
}

// Helper function to add a test to a dynamic array
inline CMUnitTest* add_test(CMUnitTest* tests, int* count, CMUnitTest test)
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

#define MICROSTRAIN_ADD_UNIT_TEST(argc, argv, test, test_suite, test_count) \
if (should_run_test(argc, argv, #test)) \
{ \
    test_suite = add_test(test_suite, test_count, cmocka_unit_test(test)); \
}

#endif // !CMOCKA_WRAPPER_H
