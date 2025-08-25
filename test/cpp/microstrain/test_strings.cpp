
extern "C" {
#include "../../c/microstrain/test_strings.h"
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

void strcat_n_wrapper_works()
{
    StringTest test;
    bool ok = microstrain::strcat_n(test.array, sizeof(test.array), &test.index, TEST_STRING, sizeof(TEST_STRING)-1);
    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void strcat_n_into_explicit_span_works()
{
    StringTest test;

    // Explicitly create a span
    bool ok = microstrain::strcat_n(test.buffer(), &test.index, TEST_STRING, sizeof(TEST_STRING)-1);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void strcat_n_into_implicit_span_works()
{
    StringTest test;

    // Implicitly create a span
    bool ok = microstrain::strcat_n({test.array, sizeof(test.array)}, &test.index, TEST_STRING, sizeof(TEST_STRING)-1);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void strcat_explicit_span_works()
{
    StringTest test;

    bool ok = microstrain::strcat(test.buffer(), &test.index, microstrain::Span<const char>{TEST_STRING});

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void strcat_explicit_string_view_works()
{
    StringTest test;

    bool ok = microstrain::strcat(test.buffer(), &test.index, std::string_view{TEST_STRING});

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void strcat_explicit_string_works()
{
    StringTest test;

    bool ok = microstrain::strcat(test.buffer(), &test.index, std::string{TEST_STRING});

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

void strcat_literal_works()
{
    StringTest test;

    bool ok = microstrain::strcat_l(test.buffer(), &test.index, "Testing!");

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_STRING, sizeof(TEST_STRING), "");
}

//
// strfmt
//

void strfmt_wrapper_works()
{
    StringTest test;

    bool ok = microstrain::strfmt(test.array, sizeof(test.array), &test.index, "%s %u %02X", "test", 100, 256);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, "test 100 100", 13, "");
}

void strfmt_into_explicit_span_works()
{
    StringTest test;

    bool ok = microstrain::strfmt(test.buffer(), &test.index, "%s %u %02X", "test", 100, 256);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, "test 100 100", 13, "");
}

void strfmt_into_implicit_span_works()
{
    StringTest test;

    bool ok = microstrain::strfmt({test.array, sizeof(test.array)}, &test.index, "%s %u %02X", "test", 100, 256);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, "test 100 100", 13, "");
}

//
// strfmt_bytes
//

void strfmt_bytes_wrapper_works()
{
    StringTest test;

    bool ok = microstrain::strfmt_bytes(test.array, sizeof(test.array), &test.index, TEST_DATA, sizeof(TEST_DATA), 2);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_DATA_STR, sizeof(TEST_DATA_STR), "");
}

void strfmt_bytes_into_span_works()
{
    StringTest test;

    bool ok = microstrain::strfmt_bytes(test.buffer(), &test.index, TEST_DATA, sizeof(TEST_DATA), 2);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_DATA_STR, sizeof(TEST_DATA_STR), "");
}

void strfmt_bytes_from_implicit_span_works()
{
    StringTest test;

    bool ok = microstrain::strfmt_bytes(test.buffer(), &test.index, {TEST_DATA}, 2);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_BUFFER_COMPARE(test.array, TEST_DATA_STR, sizeof(TEST_DATA_STR), "");
}

//
// main
//

int main()
{
    strcat_n_wrapper_works();
    strcat_n_into_explicit_span_works();
    strcat_n_into_implicit_span_works();
    strcat_explicit_span_works();
    strcat_explicit_string_view_works();
    strcat_explicit_string_works();
    // strcat_n_implicit_str_works();  // this would be ambiguous (span or string?)
    strcat_literal_works();

    strfmt_wrapper_works();
    strfmt_into_explicit_span_works();
    strfmt_into_implicit_span_works();

    strfmt_bytes_wrapper_works();
    strfmt_bytes_into_span_works();
    strfmt_bytes_from_implicit_span_works();

    return (int)g_fail_count;
}
