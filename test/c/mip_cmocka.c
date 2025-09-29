//
// Created by NDACOSTA on 9/29/2025.
//

#include "mip_cmocka.h"

static CMUnitTest* add_test(CMUnitTest* tests, int* count, CMUnitTest test)
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
