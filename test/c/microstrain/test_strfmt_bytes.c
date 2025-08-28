
#include "testutil_strings.h"

#include <microstrain/strings.h>

#include <string.h>


void format_bytes_to_empty_unterminated_buffer_works()
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    bool ok = microstrain_strfmt_bytes(buffer, 25, &index, data, sizeof(data), 0);

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_EQ(index, 8, "Index should be correct");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "A1B2C3D4", 9, "Buffer matches expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 9, "Buffer has not overrun");
}

void format_bytes_with_group1_works()
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    bool ok = microstrain_strfmt_bytes(buffer, 25, &index, data, sizeof(data), 1);

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_EQ(index, 11, "Index should be correct");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "A1 B2 C3 D4", 12, "Buffer matches expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 12, "Buffer has not overrun");
}

void format_bytes_with_group2_works()
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    bool ok = microstrain_strfmt_bytes(buffer, 25, &index, data, sizeof(data), 2);

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_EQ(index, 9, "Index should be correct");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "A1B2 C3D4", 10, "Buffer matches expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 10, "Buffer has not overrun");
}

void format_bytes_with_partial_group2_works()
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5};

    bool ok = microstrain_strfmt_bytes(buffer, 25, &index, data, sizeof(data), 2);

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_EQ(index, 12, "Index should be correct");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "A1B2 C3D4 E5", 13, "Buffer matches expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 13, "Buffer has not overrun");
}

void format_bytes_with_group4_works()
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;
    const uint8_t data[] = {0xA8, 0xB9, 0xCE, 0xDF, 0x01, 0x23, 0x45, 0x67};

    bool ok = microstrain_strfmt_bytes(buffer, 25, &index, data, sizeof(data), 4);

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_EQ(index, 17, "Index should be correct");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "A8B9CEDF 01234567", 18, "Buffer matches expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 18, "Buffer has not overrun");
}

void format_bytes_works_when_no_data()
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = microstrain_strfmt_bytes(buffer, 10, &index, NULL, 0, 0);

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_EQ(index, 0, "Index should be unchanged");
}

void format_bytes_at_offset_works()
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Data: ", 7);
    size_t index = 6;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    bool ok = microstrain_strfmt_bytes(buffer, 25, &index, data, sizeof(data), 0);

    TEST_ASSERT(ok, "Should be successful");
    TEST_ASSERT_EQ(index, 14, "Index should be correct");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Data: A1B2C3D4", 15, "Buffer matches expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 15, "Buffer has not overrun");
}

void format_bytes_at_offset_fails_gracefully_when_buffer_too_small()
{
    char buffer[50];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Data: ", 7);
    size_t index = 6;
    const uint8_t data[] = {0xA1, 0xB2, 0xC3, 0xD4};

    bool ok = microstrain_strfmt_bytes(buffer, 10, &index, data, sizeof(data), 0);

    TEST_ASSERT(!ok, "Should not be successful");
    TEST_ASSERT_EQ(index, 14, "Index should be correct");
    // Unlike other format functions, no data is written if the buffer is too small.
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Data: ", 7, "Buffer matches expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 7, "Buffer has not overrun");
    // TEST_ASSERT_BUFFER_COMPARE(buffer, "Data: A1B", 10, "Buffer matches expected result");
    // TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 10, "Buffer has not overrun");
}

int main()
{
    format_bytes_to_empty_unterminated_buffer_works();
    format_bytes_with_group1_works();
    format_bytes_with_group2_works();
    format_bytes_with_partial_group2_works();
    format_bytes_with_group4_works();
    format_bytes_works_when_no_data();
    format_bytes_at_offset_works();
    format_bytes_at_offset_fails_gracefully_when_buffer_too_small();

    return (int)g_fail_count;
}