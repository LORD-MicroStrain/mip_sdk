#pragma once

#include <unity.h>

// -----------------------------------------------------------------------------------------------------------
// Test registration
// -----------------------------------------------------------------------------------------------------------

#define INTERNAL_MICROSTRAIN_BUILD_TEST_CALLABLE_NAME(suite_name, test_name) \
    MICROSTRAIN_TEST_SUITE##suite_name##_MICROSTRAIN_TEST_CASE##test_name

#define MICROSTRAIN_TEST_CASE_IMPLEMENTATION(suite_name, test_name) \
    void INTERNAL_MICROSTRAIN_BUILD_TEST_CALLABLE_NAME(suite_name, test_name)(void)

#define MICROSTRAIN_TEST_DEFAULT_SETUP_IMPLEMENTATION()                                                      \
    void setUp(void) {}                                                                                      \
    void tearDown(void) {}                                                                                   \

#define MICROSTRAIN_TEST_BEGIN_IMPLEMENTATION() \
    UNITY_BEGIN()

#define MICROSTRAIN_TEST_END_IMPLEMENTATION() \
    UNITY_END()

// -----------------------------------------------------------------------------------------------------------
// Test execution
// -----------------------------------------------------------------------------------------------------------

#define RUN_MICROSTRAIN_TEST_CASE_IMPLEMENTATION(suite_name, test_name) \
    RUN_TEST(INTERNAL_MICROSTRAIN_BUILD_TEST_CALLABLE_NAME(suite_name, test_name))

// NOTE: Unfortunately have to reimplement and modify Unity's internals here instead
//       of using RUN_TEST/UnityDefaultTestRun since there's no other way to update
//       the file path without it being overridden.
//
//       Make sure this is up to date when updating Unity. Luckily, it doesn't look
//       like it changes often.
#define INTERNAL_RUN_MICROSTRAIN_TEST_CASE_AUTO_DISCOVER_IMPL(suite_name, test_name, file_path)  \
    do                                                                               \
    {                                                                                \
        Unity.CurrentTestName = "[TEST SUITE] " #test_name;                            \
        /* Gets the line of the calling test in the generated runner file. If the */ \
        /* test passes, this will be used for the link. If the test fails, the    */ \
        /* line number will be updated by the failing assertion and used for the  */ \
        /* link instead.                                                          */ \
        Unity.CurrentTestLineNumber = (UNITY_LINE_TYPE)__LINE__;                     \
        Unity.NumberOfTests++;                                                       \
        Unity.TestFile = file_path;                                                  \
                                                                                     \
        UNITY_CLR_DETAILS();                                                         \
        UNITY_EXEC_TIME_START();                                                     \
        UNITY_LINE_TYPE previous_failures = Unity.TestFailures;                      \
                                                                                     \
        if (TEST_PROTECT()) {                                                        \
            setUp();                                                                 \
            INTERNAL_MICROSTRAIN_BUILD_TEST_CALLABLE_NAME(suite_name, test_name)();  \
        }                                                                            \
        if (TEST_PROTECT()) {                                                        \
            tearDown();                                                              \
        }                                                                            \
                                                                                     \
        UNITY_EXEC_TIME_STOP();                                                      \
        /* If the test passes, then the line number will be for the calling test */  \
        /* in the generated runner file. We want to set it as the file in this   */  \
        /* case.                                                                 */  \
        if (Unity.TestFailures == previous_failures)                                 \
        {                                                                            \
            Unity.TestFile = __FILE__;                                               \
        }                                                                            \
        UnityConcludeTest();                                                         \
    } while(0)


/* The following wrapper macros flip the argument order from (expected, actual) to (actual, expected).
 *
 * There is no added functionality to these macros. They are simply to maintain consistency with other
 * frameworks. Assertion names remains the same, minus the "TEST_" part.
 */

// -----------------------------------------------------------------------------------------------------------
// Boolean assertions
// -----------------------------------------------------------------------------------------------------------

// TODO: Add code generation script to do the assertion argument flipping automatically

#define ASSERT_TRUE(condition) \
    TEST_ASSERT_TRUE(condition)

// -----------------------------------------------------------------------------------------------------------
// Integer assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL(actual, expected) \
    TEST_ASSERT_EQUAL_INT((expected), (actual))

#define ASSERT_EQUAL_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_INT_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_INT(actual, expected) \
    TEST_ASSERT_EQUAL_INT((expected), (actual))

#define ASSERT_EQUAL_INT_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_INT_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_INT8(actual, expected) \
    TEST_ASSERT_EQUAL_INT8((expected), (actual))

#define ASSERT_EQUAL_INT8_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_INT8_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_INT16(actual, expected) \
    TEST_ASSERT_EQUAL_INT16((expected), (actual))

#define ASSERT_EQUAL_INT16_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_INT16_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_INT32(actual, expected) \
    TEST_ASSERT_EQUAL_INT32((expected), (actual))

#define ASSERT_EQUAL_INT32_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_INT32_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_INT64(actual, expected) \
    TEST_ASSERT_EQUAL_INT64((expected), (actual))

#define ASSERT_EQUAL_INT64_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_INT64_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_UINT(actual, expected) \
    TEST_ASSERT_EQUAL_UINT((expected), (actual))

#define ASSERT_EQUAL_UINT_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_UINT_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_UINT8(actual, expected) \
    TEST_ASSERT_EQUAL_UINT8((expected), (actual))

#define ASSERT_EQUAL_UINT8_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_UINT8_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_UINT16(actual, expected) \
    TEST_ASSERT_EQUAL_UINT16((expected), (actual))

#define ASSERT_EQUAL_UINT16_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_UINT16_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_UINT32(actual, expected) \
    TEST_ASSERT_EQUAL_UINT32((expected), (actual))

#define ASSERT_EQUAL_UINT32_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_UINT32_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_UINT64(actual, expected) \
    TEST_ASSERT_EQUAL_UINT64((expected), (actual))

#define ASSERT_EQUAL_UINT64_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_UINT64_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_HEX(actual, expected) \
    TEST_ASSERT_EQUAL_HEX((expected), (actual))

#define ASSERT_EQUAL_HEX_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_HEX_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_HEX8(actual, expected) \
    TEST_ASSERT_EQUAL_HEX8((expected), (actual))

#define ASSERT_EQUAL_HEX8_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_HEX8_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_HEX16(actual, expected) \
    TEST_ASSERT_EQUAL_HEX16((expected), (actual))

#define ASSERT_EQUAL_HEX16_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_HEX16_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_HEX32(actual, expected) \
    TEST_ASSERT_EQUAL_HEX32((expected), (actual))

#define ASSERT_EQUAL_HEX32_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_HEX32_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_HEX64(actual, expected) \
    TEST_ASSERT_EQUAL_HEX64((expected), (actual))

#define ASSERT_EQUAL_HEX64_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_HEX64_MESSAGE((expected), (actual), (message))

// -----------------------------------------------------------------------------------------------------------
// Float/double assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_FLOAT(actual, expected) \
    TEST_ASSERT_EQUAL_FLOAT((expected), (actual))

#define ASSERT_EQUAL_FLOAT_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_DOUBLE(actual, expected) \
    TEST_ASSERT_EQUAL_DOUBLE((expected), (actual))

#define ASSERT_EQUAL_DOUBLE_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_DOUBLE_MESSAGE((expected), (actual), (message))

#define ASSERT_FLOAT_WITHIN(actual, delta, expected) \
    TEST_ASSERT_FLOAT_WITHIN((delta), (expected), (actual))

#define ASSERT_FLOAT_WITHIN_MESSAGE(actual, delta, expected, message) \
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE((delta), (expected), (actual), (message))

#define ASSERT_DOUBLE_WITHIN(actual, delta, expected) \
    TEST_ASSERT_DOUBLE_WITHIN((delta), (expected), (actual))

#define ASSERT_DOUBLE_WITHIN_MESSAGE(actual, delta, expected, message) \
    TEST_ASSERT_DOUBLE_WITHIN_MESSAGE((delta), (expected), (actual), (message))

// -----------------------------------------------------------------------------------------------------------
// String assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_STRING(actual, expected) \
    TEST_ASSERT_EQUAL_STRING((expected), (actual))

#define ASSERT_EQUAL_STRING_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_STRING_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_STRING_LEN(actual, expected, len) \
    TEST_ASSERT_EQUAL_STRING_LEN((expected), (actual), (len))

#define ASSERT_EQUAL_STRING_LEN_MESSAGE(actual, expected, len, message) \
    TEST_ASSERT_EQUAL_STRING_LEN_MESSAGE((expected), (actual), (len), (message))

// -----------------------------------------------------------------------------------------------------------
// Memory assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_MEMORY(actual, expected, len) \
    TEST_ASSERT_EQUAL_MEMORY((expected), (actual), (len))

#define ASSERT_EQUAL_MEMORY_MESSAGE(actual, expected, len, message) \
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE((expected), (actual), (len), (message))

// -----------------------------------------------------------------------------------------------------------
// Pointer assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_PTR(actual, expected) \
    TEST_ASSERT_EQUAL_PTR((expected), (actual))

#define ASSERT_EQUAL_PTR_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_PTR_MESSAGE((expected), (actual), (message))

// -----------------------------------------------------------------------------------------------------------
// Int array assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_INT_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_INT_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_INT_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_INT_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_INT8_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_INT8_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_INT8_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_INT8_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_INT16_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_INT16_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_INT16_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_INT16_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_INT32_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_INT32_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_INT32_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_INT32_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_INT64_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_INT64_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_INT64_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_INT64_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

// -----------------------------------------------------------------------------------------------------------
// Unsigned int array assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_UINT_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_UINT_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_UINT_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_UINT_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_UINT8_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_UINT8_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_UINT16_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_UINT16_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_UINT16_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_UINT16_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_UINT32_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_UINT32_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_UINT32_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_UINT32_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_UINT64_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_UINT64_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_UINT64_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_UINT64_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

// -----------------------------------------------------------------------------------------------------------
// Hex array assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_HEX_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_HEX_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_HEX_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_HEX_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_HEX8_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_HEX8_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_HEX16_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_HEX16_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_HEX16_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_HEX16_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_HEX32_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_HEX32_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_HEX32_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_HEX32_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_HEX64_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_HEX64_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_HEX64_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_HEX64_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

// -----------------------------------------------------------------------------------------------------------
// Float array assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_FLOAT_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_FLOAT_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_FLOAT_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_FLOAT_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

// -----------------------------------------------------------------------------------------------------------
// Double array assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_DOUBLE_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_DOUBLE_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_DOUBLE_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_DOUBLE_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

// -----------------------------------------------------------------------------------------------------------
// Pointer array assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_EQUAL_PTR_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_PTR_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_PTR_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_PTR_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

/* CHAR array assertions (if Unity supports them) */
#define ASSERT_EQUAL_CHAR_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_CHAR_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_CHAR_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_CHAR_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

// -----------------------------------------------------------------------------------------------------------
// Generic comparison assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_GREATER_THAN(actual, threshold) \
    TEST_ASSERT_GREATER_THAN((threshold), (actual))

#define ASSERT_GREATER_THAN_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN(actual, threshold) \
    TEST_ASSERT_LESS_THAN((threshold), (actual))

#define ASSERT_LESS_THAN_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE((threshold), (actual), (message))

// -----------------------------------------------------------------------------------------------------------
// Int comparison assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_GREATER_THAN_INT(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_INT((threshold), (actual))

#define ASSERT_GREATER_THAN_INT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_INT_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_INT(actual, threshold) \
    TEST_ASSERT_LESS_THAN_INT((threshold), (actual))

#define ASSERT_LESS_THAN_INT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_INT_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_INT(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_INT(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_INT((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_INT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_INT_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_INT8(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_INT8((threshold), (actual))

#define ASSERT_GREATER_THAN_INT8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_INT8_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_INT8(actual, threshold) \
    TEST_ASSERT_LESS_THAN_INT8((threshold), (actual))

#define ASSERT_LESS_THAN_INT8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_INT8_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_INT8(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT8((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_INT8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT8_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_INT8(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_INT8((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_INT8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_INT8_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_INT16(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_INT16((threshold), (actual))

#define ASSERT_GREATER_THAN_INT16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_INT16_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_INT16(actual, threshold) \
    TEST_ASSERT_LESS_THAN_INT16((threshold), (actual))

#define ASSERT_LESS_THAN_INT16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_INT16_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_INT16(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT16((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_INT16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT16_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_INT16(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_INT16((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_INT16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_INT16_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_INT32(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_INT32((threshold), (actual))

#define ASSERT_GREATER_THAN_INT32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_INT32_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_INT32(actual, threshold) \
    TEST_ASSERT_LESS_THAN_INT32((threshold), (actual))

#define ASSERT_LESS_THAN_INT32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_INT32_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_INT32(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT32((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_INT32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT32_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_INT32(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_INT32((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_INT32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_INT32_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_INT64(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_INT64((threshold), (actual))

#define ASSERT_GREATER_THAN_INT64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_INT64_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_INT64(actual, threshold) \
    TEST_ASSERT_LESS_THAN_INT64((threshold), (actual))

#define ASSERT_LESS_THAN_INT64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_INT64_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_INT64(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT64((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_INT64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_INT64_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_INT64(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_INT64((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_INT64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_INT64_MESSAGE((threshold), (actual), (message))

// -----------------------------------------------------------------------------------------------------------
// Unsigned int comparison assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_GREATER_THAN_UINT(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_UINT((threshold), (actual))

#define ASSERT_GREATER_THAN_UINT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_UINT_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_UINT(actual, threshold) \
    TEST_ASSERT_LESS_THAN_UINT((threshold), (actual))

#define ASSERT_LESS_THAN_UINT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_UINT_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_UINT(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_UINT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_UINT(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_UINT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_UINT8(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_UINT8((threshold), (actual))

#define ASSERT_GREATER_THAN_UINT8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_UINT8_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_UINT8(actual, threshold) \
    TEST_ASSERT_LESS_THAN_UINT8((threshold), (actual))

#define ASSERT_LESS_THAN_UINT8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_UINT8_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_UINT8(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_UINT8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_UINT8(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT8((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_UINT8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT8_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_UINT16(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_UINT16((threshold), (actual))

#define ASSERT_GREATER_THAN_UINT16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_UINT16_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_UINT16(actual, threshold) \
    TEST_ASSERT_LESS_THAN_UINT16((threshold), (actual))

#define ASSERT_LESS_THAN_UINT16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_UINT16_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_UINT16(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_UINT16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_UINT16(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT16((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_UINT16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT16_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_UINT32(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_UINT32((threshold), (actual))

#define ASSERT_GREATER_THAN_UINT32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_UINT32(actual, threshold) \
    TEST_ASSERT_LESS_THAN_UINT32((threshold), (actual))

#define ASSERT_LESS_THAN_UINT32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_UINT32(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_UINT32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_UINT32(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT32((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_UINT64(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_UINT64((threshold), (actual))

#define ASSERT_GREATER_THAN_UINT64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_UINT64_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_UINT64(actual, threshold) \
    TEST_ASSERT_LESS_THAN_UINT64((threshold), (actual))

#define ASSERT_LESS_THAN_UINT64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_UINT64_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_UINT64(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT64((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_UINT64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_UINT64_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_UINT64(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT64((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_UINT64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_UINT64_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_HEX8(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_HEX8((threshold), (actual))

#define ASSERT_GREATER_THAN_HEX8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_HEX8_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_HEX8(actual, threshold) \
    TEST_ASSERT_LESS_THAN_HEX8((threshold), (actual))

#define ASSERT_LESS_THAN_HEX8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_HEX8_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_HEX8(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_HEX8((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_HEX8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_HEX8_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_HEX8(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_HEX8((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_HEX8_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_HEX8_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_HEX16(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_HEX16((threshold), (actual))

#define ASSERT_GREATER_THAN_HEX16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_HEX16_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_HEX16(actual, threshold) \
    TEST_ASSERT_LESS_THAN_HEX16((threshold), (actual))

#define ASSERT_LESS_THAN_HEX16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_HEX16_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_HEX16(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_HEX16((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_HEX16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_HEX16_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_HEX16(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_HEX16((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_HEX16_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_HEX16_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_HEX32(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_HEX32((threshold), (actual))

#define ASSERT_GREATER_THAN_HEX32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_HEX32_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_HEX32(actual, threshold) \
    TEST_ASSERT_LESS_THAN_HEX32((threshold), (actual))

#define ASSERT_LESS_THAN_HEX32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_HEX32_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_HEX32(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_HEX32((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_HEX32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_HEX32_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_HEX32_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_HEX32_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_THAN_HEX64(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_HEX64((threshold), (actual))

#define ASSERT_LESS_THAN_HEX64(actual, threshold) \
    TEST_ASSERT_LESS_THAN_HEX64((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_HEX64(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_HEX64((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_HEX64(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_HEX64((threshold), (actual))

#define ASSERT_GREATER_THAN_HEX64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_HEX64_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_HEX64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_HEX64_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_HEX64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_HEX64_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_HEX64_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_HEX64_MESSAGE((threshold), (actual), (message))

// -----------------------------------------------------------------------------------------------------------
// Float comparison assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_GREATER_THAN_FLOAT(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_FLOAT((threshold), (actual))

#define ASSERT_LESS_THAN_FLOAT(actual, threshold) \
    TEST_ASSERT_LESS_THAN_FLOAT((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_FLOAT(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_FLOAT(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT((threshold), (actual))

#define ASSERT_GREATER_THAN_FLOAT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_FLOAT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_FLOAT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE((threshold), (actual), (message))

// -----------------------------------------------------------------------------------------------------------
// Double comparison assertions
// -----------------------------------------------------------------------------------------------------------

#define ASSERT_GREATER_THAN_DOUBLE(actual, threshold) \
    TEST_ASSERT_GREATER_THAN_DOUBLE((threshold), (actual))

#define ASSERT_LESS_THAN_DOUBLE(actual, threshold) \
    TEST_ASSERT_LESS_THAN_DOUBLE((threshold), (actual))

#define ASSERT_GREATER_OR_EQUAL_DOUBLE(actual, threshold) \
    TEST_ASSERT_GREATER_OR_EQUAL_DOUBLE((threshold), (actual))

#define ASSERT_LESS_OR_EQUAL_DOUBLE(actual, threshold) \
    TEST_ASSERT_LESS_OR_EQUAL_DOUBLE((threshold), (actual))

#define ASSERT_GREATER_THAN_DOUBLE_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_THAN_DOUBLE_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_THAN_DOUBLE_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_THAN_DOUBLE_MESSAGE((threshold), (actual), (message))

#define ASSERT_GREATER_OR_EQUAL_DOUBLE_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_GREATER_OR_EQUAL_DOUBLE_MESSAGE((threshold), (actual), (message))

#define ASSERT_LESS_OR_EQUAL_DOUBLE_MESSAGE(actual, threshold, message) \
    TEST_ASSERT_LESS_OR_EQUAL_DOUBLE_MESSAGE((threshold), (actual), (message))
