
#include "testutil_strings.h"

#include <microstrain/strings.h>

#include <string.h>


void string_format_on_blank_unterminated_buffer_works()
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = microstrain_strfmt(buffer, sizeof(buffer), &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT(ok, "strfmt should succeed");
    TEST_ASSERT_EQ(index, 4+2+2+4, "Index must be calculated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "4096==0x1000\0_______", sizeof(buffer), "Buffer should match expected result");
}

void string_format_fails_gracefully_when_buffer_too_small()
{
    char buffer[20];
    size_t index = 0;
    memset(buffer, '_', sizeof(buffer));

    bool ok = microstrain_strfmt(buffer, 10, &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT(!ok, "strfmt should fail");
    TEST_ASSERT_EQ(index, 4+2+2+4, "Index must be calculated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "4096==0x1\0__________", sizeof(buffer), "Buffer should match expected result");
}

void string_format_computes_size_if_buffer_null()
{
    size_t index = 0;

    bool ok = microstrain_strfmt(NULL, 0, &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT(ok, "strfmt should succeed");
    TEST_ASSERT_EQ(index, 4+2+2+4, "Index must be calculated correctly");
}

void string_format_at_offset_works()
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Test: ", 6+1);
    size_t index = 6;

    bool ok = microstrain_strfmt(buffer, sizeof(buffer), &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT(ok, "strfmt should succeed");
    TEST_ASSERT_EQ(index, 6+4+2+2+4, "Index must be calculated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Test: 4096==0x1000\0_", sizeof(buffer), "Buffer should match expected result");
}

void string_format_at_offset_fails_gracefully_if_buffer_too_small()
{
    char buffer[20];
    memset(buffer, '_', sizeof(buffer));
    memcpy(buffer, "Test: ", 6+1);
    size_t index = 6;

    bool ok = microstrain_strfmt(buffer, 10, &index, "%d==0x%x", 4096, 0x1000);

    TEST_ASSERT(!ok, "strfmt should fail");
    TEST_ASSERT_EQ(index, 6+4+2+2+4, "Index must be calculated correctly");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Test: 409\0__________", sizeof(buffer), "Buffer should match expected result");
}

void multiple_formats_work()
{
    char buffer[100];
    memset(buffer, '_', sizeof(buffer));
    size_t index = 0;

    bool ok = true;
    ok &= microstrain_strfmt(buffer, 50, &index, "Values: [");
    ok &= microstrain_strfmt(buffer, 50, &index, "A=%d, ", 54321);
    ok &= microstrain_strfmt(buffer, 50, &index, "B=0x%X, ", 0xABCD);
    ok &= microstrain_strfmt(buffer, 50, &index, "C=%s", "abcdefg");
    ok &= microstrain_strfmt(buffer, 50, &index, "]");

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
    ok &= microstrain_strfmt(buffer, 25, &index, "Values: [");
    ok &= microstrain_strfmt(buffer, 25, &index, "A=%d, ", 54321);
    ok &= microstrain_strfmt(buffer, 25, &index, "B=0x%X, ", 0xABCD);
    ok &= microstrain_strfmt(buffer, 25, &index, "C=%s", "abcdefg");
    ok &= microstrain_strfmt(buffer, 25, &index, "]");

    TEST_ASSERT(!ok, "Should not be successful");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Values: [A=54321, B=0xAB", 25, "Buffer should match expected result");
    TEST_ASSERT_BUFFER_NOT_OVERRUN(buffer, sizeof(buffer), 25, "Buffer should not be overrun");
}

int main()
{
    string_format_on_blank_unterminated_buffer_works();
    string_format_fails_gracefully_when_buffer_too_small();
    string_format_computes_size_if_buffer_null();
    string_format_at_offset_works();
    string_format_at_offset_fails_gracefully_if_buffer_too_small();
    multiple_formats_work();
    multiple_formats_fail_gracefully_when_buffer_too_small();

    return (int)g_fail_count;
}
