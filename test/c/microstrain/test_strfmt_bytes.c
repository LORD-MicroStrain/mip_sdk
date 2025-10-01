
#include "testutil_strings.h"

#include <microstrain/strings.h>

#include <string.h>







void format_bytes_works_when_no_data()
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = microstrain_string_bytes_to_hex_str(buffer, 10, &index, NULL, 0, 0);

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

    bool ok = microstrain_string_bytes_to_hex_str(buffer, 25, &index, data, sizeof(data), 0);

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

    bool ok = microstrain_string_bytes_to_hex_str(buffer, 10, &index, data, sizeof(data), 0);

    TEST_ASSERT(!ok, "Should not be successful");
    TEST_ASSERT_EQ(index, 14, "Index should be correct");
    // Unlike other format functions, no data is written if the buffer is too small.
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Data: ", 7, "Buffer matches expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 7, "Buffer has not overrun");
}

int main()
{
    format_bytes_to_blank_unterminated_buffer_works();
    format_bytes_with_group1_works_and_has_no_extra_spaces();
    format_bytes_with_group2_works_and_has_no_extra_spaces();
    format_bytes_with_group2_works_with_partial_group();
    format_bytes_with_group4_works_and_has_no_extra_spaces();
    format_bytes_works_when_no_data();
    format_bytes_at_offset_works();
    format_bytes_at_offset_fails_gracefully_when_buffer_too_small();

    return (int)g_fail_count;
}