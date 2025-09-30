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
    assert_null_terminated(buffer, 5);
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
    assert_null_terminated(buffer, 3);
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
    assert_null_terminated(buffer, 9);
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
    assert_null_terminated(buffer, 9);
    assert_string_equal(buffer, "012345678\0##########"); // TODO: I don't think characters after the null terminator are compared.
                                                          //       Should they be?
}

MICROSTRAIN_TEST_CASE(String_concatenation_to_a_non_empty_buffer_fails_gracefully_if_index_is_at_the_end)
{
    char buffer[20];
    memcpy(buffer, "012345678\0__________", 20);
    size_t index = 12;
    const size_t fake_buffer_size = 9;
    const char* msg = "ABCDEF";
    const size_t msg_size = strlen(msg);

    const bool ok = microstrain_string_concat(buffer, fake_buffer_size, &index, msg, msg_size);

    assert_false(ok);
    assert_int_equal(index, 18);
    assert_null_terminated(buffer, 9);
    assert_string_equal(buffer, "012345678\0__________");
}

MICROSTRAIN_TEST_CASE(Multiple_string_concatenations_work)
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

    assert_true(ok);
    assert_int_equal(index, 27);
    assert_null_terminated(buffer, 27);
    assert_string_equal(buffer, "This is a very long test...");
}

MICROSTRAIN_TEST_CASE(Multiple_string_concatenations_fail_gracefully_when_buffer_too_small)
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

    assert_false(ok);
    assert_int_equal(index, 27);
    assert_null_terminated(buffer, 9);
    assert_string_equal(buffer, "This is a");
}

MICROSTRAIN_TEST_CASE(A_string_can_be_formatted_and_written_to_an_empty_buffer)
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    const bool ok = microstrain_string_format(buffer, sizeof(buffer), &index, "%d==0x%x", 4096, 0x1000);

    assert_true(ok);
    assert_int_equal(index, 4+2+2+4);
    assert_null_terminated(buffer, 12);
    assert_string_equal(buffer, "4096==0x1000\0_______");
}

/*
void string_format_fails_gracefully_when_buffer_too_small()
{
    char buffer[20];
    size_t index = 0;
    memset(buffer, '_', sizeof(buffer));

    bool ok = microstrain_string_format(buffer, 10, &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT(!ok, "format should fail");
    TEST_ASSERT_EQ(index, 4+2+2+4, "Index must be calculated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "4096==0x1\0__________", sizeof(buffer), "Buffer should match expected result");
}

void string_format_computes_size_if_buffer_null()
{
    size_t index = 0;

    bool ok = microstrain_string_format(NULL, 0, &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT(ok, "format should succeed");
    TEST_ASSERT_EQ(index, 4+2+2+4, "Index must be calculated correctly");
}

void string_format_at_offset_works()
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Test: ", 6+1);
    size_t index = 6;

    bool ok = microstrain_string_format(buffer, sizeof(buffer), &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT(ok, "format should succeed");
    TEST_ASSERT_EQ(index, 6+4+2+2+4, "Index must be calculated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Test: 4096==0x1000\0_", sizeof(buffer), "Buffer should match expected result");
}

void string_format_at_offset_fails_gracefully_if_buffer_too_small()
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Test: ", 6+1);
    size_t index = 6;

    bool ok = microstrain_string_format(buffer, 10, &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT(!ok, "format should fail");
    TEST_ASSERT_EQ(index, 6+4+2+2+4, "Index must be calculated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Test: 409\0__________", sizeof(buffer), "Buffer should match expected result");
}

void multiple_formats_work()
{
    char buffer[100];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = true;
    ok &= microstrain_string_format(buffer, 50, &index, "Values: [");
    ok &= microstrain_string_format(buffer, 50, &index, "A=%d, ", 54321);
    ok &= microstrain_string_format(buffer, 50, &index, "B=0x%X, ", 0xABCD);
    ok &= microstrain_string_format(buffer, 50, &index, "C=%s", "abcdefg");
    ok &= microstrain_string_format(buffer, 50, &index, "]");

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Values: [A=54321, B=0xABCD, C=abcdefg]", 39, "Buffer should match expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 39, "Buffer should not be overrun");
}

void multiple_formats_fail_gracefully_when_buffer_too_small()
{
    char buffer[100];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = true;
    ok &= microstrain_string_format(buffer, 25, &index, "Values: [");
    ok &= microstrain_string_format(buffer, 25, &index, "A=%d, ", 54321);
    ok &= microstrain_string_format(buffer, 25, &index, "B=0x%X, ", 0xABCD);
    ok &= microstrain_string_format(buffer, 25, &index, "C=%s", "abcdefg");
    ok &= microstrain_string_format(buffer, 25, &index, "]");

    TEST_ASSERT(!ok, "Should not be successful");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Values: [A=54321, B=0xAB", 25, "Buffer should match expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 25, "Buffer should not be overrun");
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
    MICROSTRAIN_TEST_ADD(string_tests, String_concatenation_to_a_non_empty_buffer_fails_gracefully_if_index_is_at_the_end);
    MICROSTRAIN_TEST_ADD(string_tests, Multiple_string_concatenations_work);
    MICROSTRAIN_TEST_ADD(string_tests, Multiple_string_concatenations_fail_gracefully_when_buffer_too_small);
    MICROSTRAIN_TEST_SUITE_RUN("String Tests", string_tests);
    MICROSTRAIN_TEST_SUITE_END(string_tests);

    MICROSTRAIN_TEST_SUITE_START(string_formatting);
    MICROSTRAIN_TEST_ADD(string_formatting, A_string_can_be_formatted_and_written_to_an_empty_buffer);
    /*
    string_format_fails_gracefully_when_buffer_too_small();
    string_format_computes_size_if_buffer_null();
    string_format_at_offset_works();
    string_format_at_offset_fails_gracefully_if_buffer_too_small();
    multiple_formats_work();
    multiple_formats_fail_gracefully_when_buffer_too_small();
    */
    MICROSTRAIN_TEST_SUITE_RUN("String Formatting", string_formatting);
    MICROSTRAIN_TEST_SUITE_END(string_formatting);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}
