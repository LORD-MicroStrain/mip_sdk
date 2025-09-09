#define MICROSTRAIN_HAS_STD_STRING 1

#include <string_view>

#include <framework_wrappers.hpp>
#include <microstrain/strings.hpp>


static constexpr char FAKE_STRING[] = "Testing!";
static const uint8_t TEST_DATA[] = { 0x0F, 0x2E, 0x4D, 0x6C, 0x8B, 0xAA };
static const char TEST_DATA_STR[] = "0F2E 4D6C 8BAA";


/*
struct StringTest
{
    char array[1024] = {0};
    size_t index = 0;

    const microstrain::Span<char> buffer() { return {array}; }
};
*/

// TODO: Figure out naming for these tests


TEST("Span string concatenation", "A C string can be concatenated to an explicitly created span")
{
    char buffer[1024];
    microstrain::Span<char> buffer_span{buffer};
    size_t index = 0;

    // Explicitly create a span
    const bool ok = microstrain::strings::concat(buffer_span, &index, FAKE_STRING, sizeof(FAKE_STRING) - 1);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer, FAKE_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A C string can be concatenated to an implicitly created span")
{
    char buffer[1024];
    size_t index = 0;

    const bool ok = microstrain::strings::concat({buffer, sizeof(buffer)}, &index, FAKE_STRING, sizeof(FAKE_STRING)-1);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer, FAKE_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A span can be concatenated to another span")
{
    char buffer[1024];
    microstrain::Span<char> buffer_span{buffer};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer_span, &index, microstrain::Span<const char>{FAKE_STRING});

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer, FAKE_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A string view can be concatenated to a span")
{
    char buffer[1024];
    microstrain::Span<char> buffer_span{buffer};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer_span, &index, std::string_view{FAKE_STRING});

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer, FAKE_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A string can be concatenated to a span")
{
    char buffer[1024];
    microstrain::Span<char> buffer_span{buffer};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer_span, &index, std::string{FAKE_STRING});

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer, FAKE_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A C string can be fully concatenated to a span when null terminator is at the end")
{
    char buffer[1024];
    microstrain::Span<char> buffer_span{buffer};
    size_t index = 0;

    const bool ok = microstrain::strings::concat_z(buffer_span, &index, FAKE_STRING);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer, FAKE_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A C string is partially concatenated to a span when a max length is given")
{
    char buffer[1024];
    microstrain::Span<char> buffer_span{buffer};
    size_t index = 0;
    constexpr size_t character_limit = 4;

    const bool ok = microstrain::strings::concat_z(buffer_span, &index, FAKE_STRING, character_limit);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer, "Test");
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), num_chars, "");
}

TEST("Span string concatenation", "A string literal can be concatenated to a span")
{
    char buffer[1024];
    microstrain::Span<char> buffer_span{buffer};
    size_t index = 0;

    const bool ok = microstrain::strings::concat_l(buffer_span, &index, "Testing!");

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer, FAKE_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string formatting", "An span string can be formatted properly")
{
    char buffer[1024];
    microstrain::Span<char> buffer_span{buffer};
    size_t index = 0;

    const bool ok = microstrain::strings::format(buffer_span, &index, "%s %u %02X", "test", 100, 256);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer, "test 100 100");
}

/*
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
*/
