
extern "C" {
#include "../../c/microstrain/testutil_strings.h"
}

#define MICROSTRAIN_HAS_STD_STRING 1
#include <microstrain/strings.hpp>


static const char TEST_STRING[] = "Testing!";
static const uint8_t TEST_DATA[] = { 0x0F, 0x2E, 0x4D, 0x6C, 0x8B, 0xAA };
static const char TEST_DATA_STR[] = "0F2E 4D6C 8BAA";


struct StringTest
{
    char array[1024] = {0};
    size_t index = 0;

    const microstrain::Span<char> buffer() { return {array}; }
};


//
// strcat
//

void concat_works()
{
    StringTest test;

    // Explicitly create a span
    bool ok = microstrain::strings::concat(test.buffer(), &test.index, TEST_STRING, sizeof(TEST_STRING)-1);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void concat_into_implicit_span_works()
{
    StringTest test;

    // Implicitly create a span
    bool ok = microstrain::strings::concat({test.array, sizeof(test.array)}, &test.index, TEST_STRING, sizeof(TEST_STRING)-1);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void concat_explicit_span_works()
{
    StringTest test;

    bool ok = microstrain::strings::concat(test.buffer(), &test.index, microstrain::Span<const char>{TEST_STRING});

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void concat_explicit_string_view_works()
{
    StringTest test;

    bool ok = microstrain::strings::concat(test.buffer(), &test.index, std::string_view{TEST_STRING});

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void concat_explicit_string_works()
{
    StringTest test;

    bool ok = microstrain::strings::concat(test.buffer(), &test.index, std::string{TEST_STRING});

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void concat_literal_works()
{
    StringTest test;

    bool ok = microstrain::strings::concat(test.buffer(), &test.index, "Testing!");

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

//
// General formatting
//

void format_works()
{
    StringTest test;

    bool ok = microstrain::strings::format({test.array, sizeof(test.array)}, &test.index, "%s %u %02X", "test", 100, 256);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, "test 100 100", 13, "");
}

//
// Bytes
//

void format_bytes_works()
{
    StringTest test;

    bool ok = microstrain::strings::bytesToHexStr(test.buffer(), &test.index, {TEST_DATA}, 2);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_DATA_STR, sizeof(TEST_DATA_STR), "");
}

//
// main
//

int main()
{
    concat_works();
    concat_into_implicit_span_works();
    concat_explicit_span_works();
    concat_explicit_string_view_works();
    concat_explicit_string_works();
    // strcat_n_implicit_str_works();  // this would be ambiguous (span or string?)
    concat_literal_works();

    format_works();

    format_bytes_works();

    return (int)g_fail_count;
}
