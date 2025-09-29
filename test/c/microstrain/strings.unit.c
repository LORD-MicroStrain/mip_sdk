#include "mip_cmocka.h"

#include <microstrain/strings.h>

#include <string.h>

static void string_concat_to_empty_unterminated_buffer_works(void** state)
{
    char buffer[10];
    size_t index = 0;
    memset(buffer, '_', sizeof(buffer));
    const char* const str = "12345";
    const size_t len = strlen(str);

    // Before:
    // [__________]
    //  |
    //  0,index

    bool ok = microstrain_string_concat(buffer, sizeof(buffer), &index, str, len);

    // After:
    // [12345\0____]
    //  |    |
    //  0  index

    TEST_ASSERT(ok, "strcat_n should succeed");
    TEST_ASSERT_EQ(index, len, "Index must be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "12345\0____", sizeof(buffer), "Buffer should match expected result");
}

static void string_concat_fails_gracefully_if_buffer_too_small(void** state)
{
    char buffer[10];
    const size_t fake_buffer_size = 4;  // dummy so we can check if the buffer overran
    size_t index = 0;
    memset(buffer, '_', sizeof(buffer));
    const char* const str = "12345";
    const size_t len = strlen(str);

    // Before:
    // [__________]
    //  |  |
    //  0  size

    bool ok = microstrain_string_concat(buffer, fake_buffer_size, &index, str, len);

    // After:
    // [123\0______]
    //  |  |
    //  0  size

    TEST_ASSERT(!ok, "strcat_n should fail");
    TEST_ASSERT_EQ(index, len, "Index must still be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "123\0______", sizeof(buffer), "Buffer contents are as expected");
}

static void string_concat_computes_size_if_buffer_null(void** state)
{
    size_t index = 0;
    const char* const str = "12345";
    const size_t len = strlen(str);

    bool ok = microstrain_string_concat(NULL, 0, &index, str, len);

    TEST_ASSERT(ok, "strcat_n should succeed");
    TEST_ASSERT_EQ(index, len, "Index should be correct");
}

static void string_concat_at_offset_works(void** state)
{
    char buffer[10];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "12345", 6);
    size_t index = 5;

    // Before:
    // [12345\0____]
    //  |    |
    //  0    index

    bool ok = microstrain_string_concat(buffer, sizeof(buffer), &index, "6789", 4);

    // After:
    // [123456789\0]
    //  |        |
    //  0      index

    TEST_ASSERT(ok, "strcat_n should succeed");
    TEST_ASSERT_EQ(index, 9, "Index must be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "123456789\0", sizeof(buffer), "Buffer should match expected result");
}

static void string_concat_at_offset_fails_gracefully_if_buffer_too_small(void** state)
{
    char buffer[20];
    const size_t fake_buffer_size = 10;
    memcpy(buffer, "01234567\0_##########", 20);
    size_t index = 8;

    bool ok = microstrain_string_concat(buffer, fake_buffer_size, &index, "89ABCDEF", 8);

    TEST_ASSERT(!ok, "strcat_n should fail");
    TEST_ASSERT_EQ(index, 16, "Index must be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "012345678\0##########", sizeof(buffer), "Buffer should match expected result");
}

static void string_concat_at_offset_fails_gracefully_if_index_at_end(void** state)
{
    char buffer[20];
    memcpy(buffer, "012345678\0__________", 20);
    size_t index = 12;
    const size_t fake_buffer_size = 9;
    const char* msg = "ABCDEF";
    const size_t msg_size = strlen(msg);

    bool ok = microstrain_string_concat(buffer, fake_buffer_size, &index, msg, msg_size);

    TEST_ASSERT(!ok, "strcat_n should fail");
    TEST_ASSERT_EQ(index, 18, "Index must be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "012345678\0__________", sizeof(buffer), "Buffer should match expected result");
}

static void multiple_string_concats_work(void** state)
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = true;
    ok &= microstrain_string_concat_l(buffer, sizeof(buffer), &index, "This ");
    ok &= microstrain_string_concat_l(buffer, sizeof(buffer), &index, "is ");
    ok &= microstrain_string_concat_l(buffer, sizeof(buffer), &index, "a ");
    ok &= microstrain_string_concat_l(buffer, sizeof(buffer), &index, "very ");
    ok &= microstrain_string_concat_l(buffer, sizeof(buffer), &index, "long ");
    ok &= microstrain_string_concat_l(buffer, sizeof(buffer), &index, "test...");

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_EQ(index, 27, "Index should be the total number of chars");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "This is a very long test...", 27, "Buffer should have entire string");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 28, "Buffer has not overrun");
}

static void multiple_string_concats_fail_gracefully_when_buffer_too_small(void** state)
{
    char buffer[50];
    const size_t fake_buffer_size = 10;
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = true;
    ok &= microstrain_string_concat_l(buffer, fake_buffer_size, &index, "This ");
    ok &= microstrain_string_concat_l(buffer, fake_buffer_size, &index, "is ");
    ok &= microstrain_string_concat_l(buffer, fake_buffer_size, &index, "a ");
    ok &= microstrain_string_concat_l(buffer, fake_buffer_size, &index, "very ");
    ok &= microstrain_string_concat_l(buffer, fake_buffer_size, &index, "long ");
    ok &= microstrain_string_concat_l(buffer, fake_buffer_size, &index, "test...");

    TEST_ASSERT(!ok, "Should not be successful");
    TEST_ASSERT_EQ(index, 27, "Index should be the total number of chars");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "This is a", 10, "Buffer should have partial string w/ terminator");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 10, "Buffer has not overrun");
}

int main()
{
    MICROSTRAIN_UNIT_TESTS_INIT;

    MICROSTRAIN_TEST_SUITE_START(string_tests);

    MICROSTRAIN_ADD_UNIT_TEST(string_tests, string_concat_to_empty_unterminated_buffer_works);
    MICROSTRAIN_ADD_UNIT_TEST(string_tests, string_concat_fails_gracefully_if_buffer_too_small);
    MICROSTRAIN_ADD_UNIT_TEST(string_tests, string_concat_computes_size_if_buffer_null);
    MICROSTRAIN_ADD_UNIT_TEST(string_tests, string_concat_at_offset_works);
    MICROSTRAIN_ADD_UNIT_TEST(string_tests, string_concat_at_offset_fails_gracefully_if_buffer_too_small);
    MICROSTRAIN_ADD_UNIT_TEST(string_tests, string_concat_at_offset_fails_gracefully_if_index_at_end);
    MICROSTRAIN_ADD_UNIT_TEST(string_tests, multiple_string_concats_work);
    MICROSTRAIN_ADD_UNIT_TEST(string_tests, multiple_string_concats_fail_gracefully_when_buffer_too_small);

    MICROSTRAIN_TEST_SUITE_RUN("String Tests", string_tests);

    MICROSTRAIN_TEST_SUITE_END(string_tests);

    return MICROSTRAIN_UNIT_TEST_FAILURES;
}
