#include "mip_cmocka.h"

#include <microstrain/strings.h>

#include <string.h>

MICROSTRAIN_TEST_CASE(A_zero_terminated_string_can_be_concatenated_to_an_empty_buffer)
{
    char buffer[10];
    size_t index = 0;
    memset(buffer, '_', sizeof(buffer));
    const char* const string = "12345";
    const size_t string_length = strlen(string);

    const bool ok = microstrain_string_concat(buffer, sizeof(buffer), &index, string, string_length);

    assert_true(ok);
    assert_int_equal(index, string_length);
    assert_char_equal(buffer[5], '\0');
    assert_string_equal(buffer, "12345\0____");
}

MICROSTRAIN_TEST_CASE(String_concatenation_to_an_empty_buffer_fails_gracefully_if_buffer_too_small)
{
    char buffer[10];
    const size_t fake_buffer_size = 4;  // dummy so we can check if the buffer overran
    size_t index = 0;
    memset(buffer, '_', sizeof(buffer));
    const char* const string = "12345";
    const size_t string_length = strlen(string);

    const bool ok = microstrain_string_concat(buffer, fake_buffer_size, &index, string, string_length);

    assert_false(ok);
    assert_int_equal(index, string_length);
    assert_char_equal(buffer[3], '\0');
    assert_string_equal(buffer, "123\0______");
}

MICROSTRAIN_TEST_CASE(String_concatenation_automatically_computes_size_when_buffer_is_null)
{
    size_t index = 0;
    const char* const string = "12345";
    const size_t string_length = strlen(string);

    const bool ok = microstrain_string_concat(NULL, 0, &index, string, string_length);

    assert_true(ok);
    assert_int_equal(index, string_length);
}

MICROSTRAIN_TEST_CASE(A_zero_terminated_string_can_be_concatenated_to_a_non_empty_buffer)
{
    char buffer[10];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "12345", 6);
    size_t index = 5;

    const bool ok = microstrain_string_concat(buffer, sizeof(buffer), &index, "6789", 4);

    assert_true(ok);
    assert_int_equal(index, 9);
    assert_char_equal(buffer[9], '\0');
    assert_string_equal(buffer, "123456789\0");
}

MICROSTRAIN_TEST_CASE(String_concatenation_to_a_non_empty_buffer_fails_gracefully_if_buffer_too_small)
{
    char buffer[20];
    const size_t fake_buffer_size = 10;
    memcpy(buffer, "01234567\0_##########", 20);
    size_t index = 8;

    const bool ok = microstrain_string_concat(buffer, fake_buffer_size, &index, "89ABCDEF", 8);

    assert_false(ok);
    assert_int_equal(index, 16);
    assert_char_equal(buffer[9], '\0');
    assert_string_equal(buffer, "012345678\0##########"); // TODO: I don't think characters after the null terminator are compared.
                                                          //       Should they be?
}

/*
MICROSTRAIN_TEST_CASE(string_concat_at_offset_fails_gracefully_if_index_at_end)
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

MICROSTRAIN_TEST_CASE(multiple_string_concats_work)
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

MICROSTRAIN_TEST_CASE(multiple_string_concats_fail_gracefully_when_buffer_too_small)
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

*/
int main()
{
    MICROSTRAIN_TEST_INIT;

    MICROSTRAIN_TEST_SUITE_START(string_tests);

    MICROSTRAIN_TEST_ADD(string_tests, A_zero_terminated_string_can_be_concatenated_to_an_empty_buffer);
    MICROSTRAIN_TEST_ADD(string_tests, String_concatenation_to_an_empty_buffer_fails_gracefully_if_buffer_too_small);
    MICROSTRAIN_TEST_ADD(string_tests, String_concatenation_automatically_computes_size_when_buffer_is_null);
    MICROSTRAIN_TEST_ADD(string_tests, A_zero_terminated_string_can_be_concatenated_to_a_non_empty_buffer);
    MICROSTRAIN_TEST_ADD(string_tests, String_concatenation_to_a_non_empty_buffer_fails_gracefully_if_buffer_too_small);
    /*
    MICROSTRAIN_TEST_ADD(string_tests, string_concat_at_offset_fails_gracefully_if_index_at_end);
    MICROSTRAIN_TEST_ADD(string_tests, multiple_string_concats_work);
    MICROSTRAIN_TEST_ADD(string_tests, multiple_string_concats_fail_gracefully_when_buffer_too_small);
    */

    MICROSTRAIN_TEST_SUITE_RUN("String Tests", string_tests);

    MICROSTRAIN_TEST_SUITE_END(string_tests);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}
