#define MICROSTRAIN_HAS_STD_STRING 1

#include <string_view>

#include <microstrain_test.hpp>
#include <microstrain/strings.hpp>

static constexpr char CHECK_STRING[] = "Test: \"quotes\" 'single' & <xml/> {json} [array] $100 €50 ¥200 @user #tag 🚀 \n\t\\ 100% café naïve";
static constexpr size_t CHECK_STRING_LENGTH = 102; // Length without null terminator

// TODO: Figure out naming for these tests

TEST("Span string concatenation", "A C string can be concatenated to a span")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer, &index, CHECK_STRING, CHECK_STRING_LENGTH);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(buffer, CHECK_STRING_LENGTH);
}

TEST("Span string concatenation", "A span can be concatenated to another span")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer, &index, microstrain::Span<const char>{CHECK_STRING});

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(buffer, CHECK_STRING_LENGTH);
}

TEST("Span string concatenation", "A string view can be concatenated to a span")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer, &index, std::string_view{CHECK_STRING});

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(buffer, CHECK_STRING_LENGTH);
}

TEST("Span string concatenation", "A string can be concatenated to a span")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer, &index, std::string{CHECK_STRING});

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(buffer, CHECK_STRING_LENGTH);
}

TEST("Span string concatenation", "A C string can be fully concatenated to a span when null terminator is at the end")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat_z(buffer, &index, CHECK_STRING);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(buffer, CHECK_STRING_LENGTH);
}

TEST("Span string concatenation", "A C string is partially concatenated to a span when a max length is given")
{
    char buffer[1024] = {};
    size_t index = 0;
    constexpr size_t character_limit = 4;

    const bool ok = microstrain::strings::concat_z(buffer, &index, "123456789", character_limit);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, "1234");
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(buffer, character_limit);
}

TEST("Span string concatenation", "A string literal can be concatenated to a span")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat_l(buffer, &index, "123456789");

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, "123456789");
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(buffer, 9);
}

TEST("Span string formatting", "A span string can be formatted properly")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::format(buffer, &index, "%s %u %02X", "test", 100, 256);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, "test 100 100");
}

TEST("Byte formatting", "A byte array can be formatted to a text buffer in hexadecimal")
{
    char buffer[1024] = {};
    size_t index = 0;
    const uint8_t DATA[] = { 0x0F, 0x2E, 0x4D, 0x6C, 0x8B, 0xAA };

    const bool ok = microstrain::strings::bytesToHexStr(buffer, &index, DATA, 2);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, "0F2E 4D6C 8BAA");
}
