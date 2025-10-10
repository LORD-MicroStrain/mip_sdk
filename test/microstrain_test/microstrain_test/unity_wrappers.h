#pragma once

#include <unity.h>

/* The following wrapper macros flip the argument order from (expected, actual) to (actual, expected).
 *
 * There is no added functionality to these macros. They are simply to maintain consistency with other
 * frameworks. Assertion names remains the same, minus the "TEST_" part.
 */

/* Boolean assertions */
#define ASSERT_TRUE(condition) \
    TEST_ASSERT_TRUE(condition)

/* Integer assertions */
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

/* Float/Double assertions */
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

/* String assertions */
#define ASSERT_EQUAL_STRING(actual, expected) \
    TEST_ASSERT_EQUAL_STRING((expected), (actual))

#define ASSERT_EQUAL_STRING_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_STRING_MESSAGE((expected), (actual), (message))

#define ASSERT_EQUAL_STRING_LEN(actual, expected, len) \
    TEST_ASSERT_EQUAL_STRING_LEN((expected), (actual), (len))

#define ASSERT_EQUAL_STRING_LEN_MESSAGE(actual, expected, len, message) \
    TEST_ASSERT_EQUAL_STRING_LEN_MESSAGE((expected), (actual), (len), (message))

/* Memory assertions */
#define ASSERT_EQUAL_MEMORY(actual, expected, len) \
    TEST_ASSERT_EQUAL_MEMORY((expected), (actual), (len))

#define ASSERT_EQUAL_MEMORY_MESSAGE(actual, expected, len, message) \
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE((expected), (actual), (len), (message))

/* Pointer assertions */
#define ASSERT_EQUAL_PTR(actual, expected) \
    TEST_ASSERT_EQUAL_PTR((expected), (actual))

#define ASSERT_EQUAL_PTR_MESSAGE(actual, expected, message) \
    TEST_ASSERT_EQUAL_PTR_MESSAGE((expected), (actual), (message))

/* Array assertions */
#define ASSERT_EQUAL_INT_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_INT_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_INT_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_INT_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_UINT_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_UINT_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_UINT_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_UINT_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_HEX_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_HEX_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_HEX_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_HEX_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_FLOAT_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_FLOAT_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_FLOAT_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_FLOAT_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

#define ASSERT_EQUAL_DOUBLE_ARRAY(actual, expected, num_elements) \
    TEST_ASSERT_EQUAL_DOUBLE_ARRAY((expected), (actual), (num_elements))

#define ASSERT_EQUAL_DOUBLE_ARRAY_MESSAGE(actual, expected, num_elements, message) \
    TEST_ASSERT_EQUAL_DOUBLE_ARRAY_MESSAGE((expected), (actual), (num_elements), (message))

/* Generic comparisons */
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

/* INT comparisons */
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

/* INT8 comparisons */
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

/* INT16 comparisons */
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

/* INT32 comparisons */
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

/* INT64 comparisons */
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

/* UINT comparisons */
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

/* UINT8 comparisons */
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

/* UINT16 comparisons */
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

/* UINT32 comparisons */
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

/* UINT64 comparisons */
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

/* HEX comparisons */
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

/* FLOAT comparisons */
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

/* DOUBLE comparisons */
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
