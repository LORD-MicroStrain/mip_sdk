#include <string.h>

#include <microstrain/strings.h>
#include <unity.h>

// TODO: Switch to use test suites when standardized test registration interface is implemented

void setUp(void) {}
void tearDown(void) {}

void A_zero_terminated_string_can_be_concatenated_to_an_empty_buffer(void)
{
    char buffer[10];
    size_t index = 0;
    memset(buffer, '_', sizeof(buffer));
    const char* const string = "12345";
    const size_t string_length = strlen(string);

    const bool ok = microstrain_string_concat(buffer, sizeof(buffer), &index, string, string_length);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, string_length);
    TEST_ASSERT_EQUAL_CHAR(buffer[5], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "12345\0____");
}

void String_concatenation_to_an_empty_buffer_fails_gracefully_if_buffer_too_small(void)
{
    char buffer[10];
    const size_t fake_buffer_size = 4;  // dummy so we can check if the buffer overran
    size_t index = 0;
    memset(buffer, '_', sizeof(buffer));
    const char* const string = "12345";
    const size_t string_length = strlen(string);

    const bool ok = microstrain_string_concat(buffer, fake_buffer_size, &index, string, string_length);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(index, string_length);
    TEST_ASSERT_EQUAL_CHAR(buffer[3], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "123\0______");
}

void String_concatenation_computes_required_buffer_size_when_buffer_is_null(void)
{
    size_t index = 0;
    const char* const string = "12345";
    const size_t string_length = strlen(string);

    const bool ok = microstrain_string_concat(NULL, 0, &index, string, string_length);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, string_length);
}

void A_zero_terminated_string_can_be_concatenated_to_a_non_empty_buffer(void)
{
    char buffer[10];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "12345", 6);
    size_t index = 5;

    const bool ok = microstrain_string_concat(buffer, sizeof(buffer), &index, "6789", 4);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 9);
    TEST_ASSERT_EQUAL_CHAR(buffer[9], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "123456789\0");
}

void String_concatenation_to_a_non_empty_buffer_fails_gracefully_if_buffer_too_small(void)
{
    char buffer[20];
    const size_t fake_buffer_size = 10;
    memcpy(buffer, "01234567\0_##########", 20);
    size_t index = 8;

    const bool ok = microstrain_string_concat(buffer, fake_buffer_size, &index, "89ABCDEF", 8);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(index, 16);
    TEST_ASSERT_EQUAL_CHAR(buffer[9], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "012345678\0##########"); // TODO: I don't think characters after the null terminator are compared.
                                                               //       Should they be?

}

void String_concatenation_to_a_non_empty_buffer_fails_gracefully_if_index_is_at_the_end(void)
{
    char buffer[20];
    memcpy(buffer, "012345678\0__________", 20);
    size_t index = 12;
    const size_t fake_buffer_size = 9;
    const char* msg = "ABCDEF";
    const size_t msg_size = strlen(msg);

    const bool ok = microstrain_string_concat(buffer, fake_buffer_size, &index, msg, msg_size);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(index, 18);
    TEST_ASSERT_EQUAL_CHAR(buffer[9], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "012345678\0__________");
}

void Multiple_string_concatenations_work(void)
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

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 27);
    TEST_ASSERT_EQUAL_CHAR(buffer[27], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "This is a very long test...");
}

void Multiple_string_concatenations_fail_gracefully_when_buffer_too_small(void)
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

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(index, 27);
    TEST_ASSERT_EQUAL_CHAR(buffer[9], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "This is a");
}

void A_string_can_be_formatted_and_written_to_an_empty_buffer(void)
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    const bool ok = microstrain_string_format(buffer, sizeof(buffer), &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 4+2+2+4);
    TEST_ASSERT_EQUAL_CHAR(buffer[12], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "4096==0x1000\0_______");
}

void String_formatting_fails_gracefully_when_buffer_is_too_small(void)
{
    char buffer[20];
    size_t index = 0;
    memset(buffer, '_', sizeof(buffer));

    const bool ok = microstrain_string_format(buffer, 10, &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(index, 4+2+2+4);
    TEST_ASSERT_EQUAL_CHAR(buffer[9], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "4096==0x1\0__________");
}

void String_formatting_calculates_required_buffer_size_if_buffer_is_null(void)
{
    size_t index = 0;

    const bool ok = microstrain_string_format(NULL, 0, &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 4+2+2+4);
}

void A_string_can_be_formatted_and_written_to_a_non_empty_buffer(void)
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Test: ", 6+1);
    size_t index = 6;

    const bool ok = microstrain_string_format(buffer, sizeof(buffer), &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 6+4+2+2+4);
    TEST_ASSERT_EQUAL_CHAR(buffer[18], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "Test: 4096==0x1000\0_");
}

void String_formatting_to_a_non_empty_buffer_fails_gracefully_if_buffer_too_small(void)
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Test: ", 6+1);
    size_t index = 6;

    const bool ok = microstrain_string_format(buffer, 10, &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(index, 6+4+2+2+4);
    TEST_ASSERT_EQUAL_CHAR(buffer[9], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "Test: 409\0__________");
}

void Multiple_string_formattings_work(void)
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

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 9+2+5+2+4+4+2+2+7+1);
    TEST_ASSERT_EQUAL_CHAR(buffer[38], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "Values: [A=54321, B=0xABCD, C=abcdefg]");
}

void Multiple_string_formattings_fail_gracefully_when_buffer_too_small(void)
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

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(index, 9+2+5+2+4+4+2+2+7+1);
    TEST_ASSERT_EQUAL_CHAR(buffer[24], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "Values: [A=54321, B=0xAB");
}

void A_byte_array_can_formatted_as_hexadecimal_and_written_to_a_string_buffer(void)
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    const bool ok = microstrain_string_bytes_to_hex_str(buffer, 25, &index, data, sizeof(data), 0);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 2+2+2+2);
    TEST_ASSERT_EQUAL_CHAR(buffer[8], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "A1B2C3D4");
}

void A_byte_array_can_formatted_as_hexadecimal_with_group1_and_written_to_a_string_buffer(void)
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    const bool ok = microstrain_string_bytes_to_hex_str(buffer, 25, &index, data, sizeof(data), 1);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 2+1+2+1+2+1+2);
    TEST_ASSERT_EQUAL_CHAR(buffer[11], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "A1 B2 C3 D4");
}

void A_byte_array_can_formatted_as_hexadecimal_with_group2_and_written_to_a_string_buffer(void)
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    const bool ok = microstrain_string_bytes_to_hex_str(buffer, 25, &index, data, sizeof(data), 2);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 2+2+1+2+2);
    TEST_ASSERT_EQUAL_CHAR(buffer[9], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "A1B2 C3D4");
}

void A_byte_array_can_formatted_as_hexadecimal_with_partial_group2_and_written_to_a_string_buffer(void)
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5};

    const bool ok = microstrain_string_bytes_to_hex_str(buffer, 25, &index, data, sizeof(data), 2);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 2+2+1+2+2+1+2);
    TEST_ASSERT_EQUAL_CHAR(buffer[12], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "A1B2 C3D4 E5");
}

void A_byte_array_can_formatted_as_hexadecimal_with_group4_and_written_to_a_string_buffer(void)
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA8, 0xB9, 0xCE, 0xDF, 0x01, 0x23, 0x45, 0x67};

    const bool ok = microstrain_string_bytes_to_hex_str(buffer, 25, &index, data, sizeof(data), 4);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 2+2+2+2+1+2+2+2+2);
    TEST_ASSERT_EQUAL_CHAR(buffer[17], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "A8B9CEDF 01234567");
}

void Byte_formatting_handles_no_data_properly(void)
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    const bool ok = microstrain_string_bytes_to_hex_str(buffer, 10, &index, NULL, 0, 0);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 0);
}

void A_byte_array_can_formatted_as_hexadecimal_and_written_to_a_non_empty_string_buffer(void)
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Data: ", 7);
    size_t index = 6;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    const bool ok = microstrain_string_bytes_to_hex_str(buffer, 25, &index, data, sizeof(data), 0);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(index, 6+2+2+2+2);
    TEST_ASSERT_EQUAL_CHAR(buffer[14], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "Data: A1B2C3D4");
}

void Byte_formatting_fails_gracefully_when_buffer_too_small(void)
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Data: ", 7);
    size_t index = 6;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    const bool ok = microstrain_string_bytes_to_hex_str(buffer, 10, &index, data, sizeof(data), 0);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(index, 6+2+2+2+2);
    TEST_ASSERT_EQUAL_CHAR(buffer[6], '\0');
    TEST_ASSERT_EQUAL_STRING(buffer, "Data: ");
}

int main()
{
    UNITY_BEGIN();

    // Suite: string concatenation
    RUN_TEST(A_zero_terminated_string_can_be_concatenated_to_an_empty_buffer);
    RUN_TEST(String_concatenation_to_an_empty_buffer_fails_gracefully_if_buffer_too_small);
    RUN_TEST(String_concatenation_computes_required_buffer_size_when_buffer_is_null);
    RUN_TEST(A_zero_terminated_string_can_be_concatenated_to_a_non_empty_buffer);
    RUN_TEST(String_concatenation_to_a_non_empty_buffer_fails_gracefully_if_buffer_too_small);
    RUN_TEST(String_concatenation_to_a_non_empty_buffer_fails_gracefully_if_index_is_at_the_end);
    RUN_TEST(Multiple_string_concatenations_work);
    RUN_TEST(Multiple_string_concatenations_fail_gracefully_when_buffer_too_small);

    // Suite: string formatting
    RUN_TEST(A_string_can_be_formatted_and_written_to_an_empty_buffer);
    RUN_TEST(String_formatting_fails_gracefully_when_buffer_is_too_small);
    RUN_TEST(String_formatting_calculates_required_buffer_size_if_buffer_is_null);
    RUN_TEST(A_string_can_be_formatted_and_written_to_a_non_empty_buffer);
    RUN_TEST(String_formatting_to_a_non_empty_buffer_fails_gracefully_if_buffer_too_small);
    RUN_TEST(Multiple_string_formattings_work);
    RUN_TEST(Multiple_string_formattings_fail_gracefully_when_buffer_too_small);
    RUN_TEST(A_byte_array_can_formatted_as_hexadecimal_and_written_to_a_string_buffer);
    RUN_TEST(A_byte_array_can_formatted_as_hexadecimal_with_group1_and_written_to_a_string_buffer);
    RUN_TEST(A_byte_array_can_formatted_as_hexadecimal_with_group2_and_written_to_a_string_buffer);
    RUN_TEST(A_byte_array_can_formatted_as_hexadecimal_with_partial_group2_and_written_to_a_string_buffer);
    RUN_TEST(A_byte_array_can_formatted_as_hexadecimal_with_group4_and_written_to_a_string_buffer);
    RUN_TEST(Byte_formatting_handles_no_data_properly);
    RUN_TEST(A_byte_array_can_formatted_as_hexadecimal_and_written_to_a_non_empty_string_buffer);
    RUN_TEST(Byte_formatting_fails_gracefully_when_buffer_too_small);

    return UNITY_END();
}
