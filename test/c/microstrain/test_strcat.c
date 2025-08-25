
#include "test_strings.h"

#include <microstrain/strings.h>

#include <string.h>


void string_concat_to_empty_unterminated_buffer_works()
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

    bool ok = microstrain_strcat_n(buffer, sizeof(buffer), &index, str, len);

    // After:
    // [12345\0____]
    //  |    |
    //  0  index

    TEST_ASSERT(ok, "strcat_n should succeed");
    TEST_ASSERT_EQ(index, len, "Index must be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "12345\0____", sizeof(buffer), "Buffer should match expected result");
}

void string_concat_fails_gracefully_if_buffer_too_small()
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

    bool ok = microstrain_strcat_n(buffer, fake_buffer_size, &index, str, len);

    // After:
    // [123\0______]
    //  |  |
    //  0  size

    TEST_ASSERT(!ok, "strcat_n should fail");
    TEST_ASSERT_EQ(index, len, "Index must still be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "123\0______", sizeof(buffer), "Buffer contents are as expected");
}

void string_concat_computes_size_if_buffer_null()
{
    size_t index = 0;
    const char* const str = "12345";
    const size_t len = strlen(str);

    bool ok = microstrain_strcat_n(NULL, 0, &index, str, len);

    TEST_ASSERT(ok, "strcat_n should succeed");
    TEST_ASSERT_EQ(index, len, "Index should be correct");
}

void string_concat_at_offset_works()
{
    char buffer[10];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "12345", 6);
    size_t index = 5;

    // Before:
    // [12345\0____]
    //  |    |
    //  0    index

    bool ok = microstrain_strcat_n(buffer, sizeof(buffer), &index, "6789", 4);

    // After:
    // [123456789\0]
    //  |        |
    //  0      index

    TEST_ASSERT(ok, "strcat_n should succeed");
    TEST_ASSERT_EQ(index, 9, "Index must be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "123456789\0", sizeof(buffer), "Buffer should match expected result");
}

void string_concat_at_offset_fails_gracefully_if_buffer_too_small()
{
    char buffer[20];
    memcpy(buffer, "01234567\0_##########", 20);
    size_t index = 8;

    bool ok = microstrain_strcat_n(buffer, 10, &index, "89ABCDEF", 8);

    TEST_ASSERT(!ok, "strcat_n should fail");
    TEST_ASSERT_EQ(index, 16, "Index must be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "012345678\0##########", sizeof(buffer), "Buffer should match expected result");
}

void string_concat_at_offset_fails_gracefully_if_buffer_already_overrun()
{
    char buffer[20];
    memcpy(buffer, "012345678\0__________", 20);
    size_t index = 12;

    bool ok = microstrain_strcat_n(buffer, 10, &index, "ABCDEF", 6);

    TEST_ASSERT(!ok, "strcat_n should fail");
    TEST_ASSERT_EQ(index, 18, "Index must be updated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "012345678\0__________", sizeof(buffer), "Buffer should match expected result");
}

void multiple_strcats_work()
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = true;
    ok &= microstrain_strcat_l(buffer, sizeof(buffer), &index, "This ");
    ok &= microstrain_strcat_l(buffer, sizeof(buffer), &index, "is ");
    ok &= microstrain_strcat_l(buffer, sizeof(buffer), &index, "a ");
    ok &= microstrain_strcat_l(buffer, sizeof(buffer), &index, "very ");
    ok &= microstrain_strcat_l(buffer, sizeof(buffer), &index, "long ");
    ok &= microstrain_strcat_l(buffer, sizeof(buffer), &index, "test...");

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_EQ(index, 27, "Index should be the total number of chars");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "This is a very long test...", 27, "Buffer should have entire string");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 28, "Buffer has not overrun");
}

void multiple_strcats_fail_gracefully_when_buffer_too_small()
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = true;
    ok &= microstrain_strcat_l(buffer, 10, &index, "This ");
    ok &= microstrain_strcat_l(buffer, 10, &index, "is ");
    ok &= microstrain_strcat_l(buffer, 10, &index, "a ");
    ok &= microstrain_strcat_l(buffer, 10, &index, "very ");
    ok &= microstrain_strcat_l(buffer, 10, &index, "long ");
    ok &= microstrain_strcat_l(buffer, 10, &index, "test...");

    TEST_ASSERT(!ok, "Should not be successful");
    TEST_ASSERT_EQ(index, 27, "Index should be the total number of chars");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "This is a", 10, "Buffer should have partial string w/ terminator");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 11, "Buffer has not overrun");
}

int main()
{
    string_concat_to_empty_unterminated_buffer_works();
    string_concat_fails_gracefully_if_buffer_too_small();
    string_concat_computes_size_if_buffer_null();
    string_concat_at_offset_works();
    string_concat_at_offset_fails_gracefully_if_buffer_too_small();
    string_concat_at_offset_fails_gracefully_if_buffer_already_overrun();
    multiple_strcats_work();
    multiple_strcats_fail_gracefully_when_buffer_too_small();

    return (int)g_fail_count;
}
