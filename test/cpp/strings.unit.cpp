#define MICROSTRAIN_HAS_STD_STRING 1

#include <string_view>

#include <framework_wrappers.hpp>
#include <microstrain/strings.hpp>

static constexpr char CHECK_STRING[] = "Test: \"quotes\" 'single' & <xml/> {json} [array] $100 €50 ¥200 @user #tag 🚀 \n\t\\ 100% café naïve";
size_t CHECK_STRING_LENGTH = 102; // Length without null terminator

struct BufferWrapper
{
    char array[1024] = {};
    microstrain::Span<char> span{array};
    size_t index = 0;
};

// TODO: Figure out naming for these tests

TEST("Span string concatenation", "A C string can be concatenated to an explicitly created span")
{
    BufferWrapper buffer{};

    const bool ok = microstrain::strings::concat(buffer.span, &buffer.index, CHECK_STRING, CHECK_STRING_LENGTH);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(CHECK_STRING, buffer.array);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(CHECK_STRING_LENGTH, buffer.array);
}

TEST("Span string concatenation", "A C string can be concatenated to an implicitly created span")
{
    BufferWrapper buffer{};

    const bool ok = microstrain::strings::concat({buffer.array, sizeof(buffer.array)}, &buffer.index, CHECK_STRING, CHECK_STRING_LENGTH);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer.array, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(CHECK_STRING_LENGTH, buffer.array);
}

TEST("Span string concatenation", "A span can be concatenated to another span")
{
    BufferWrapper buffer{};

    const bool ok = microstrain::strings::concat(buffer.span, &buffer.index, microstrain::Span<const char>{CHECK_STRING});

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer.array, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(CHECK_STRING_LENGTH, buffer.array);
}

TEST("Span string concatenation", "A string view can be concatenated to a span")
{
    BufferWrapper buffer{};

    const bool ok = microstrain::strings::concat(buffer.span, &buffer.index, std::string_view{CHECK_STRING});

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer.array, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(CHECK_STRING_LENGTH, buffer.array);
}

TEST("Span string concatenation", "A string can be concatenated to a span")
{
    BufferWrapper buffer{};

    const bool ok = microstrain::strings::concat(buffer.span, &buffer.index, std::string{CHECK_STRING});

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer.array, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(CHECK_STRING_LENGTH, buffer.array);
}

TEST("Span string concatenation", "A C string can be fully concatenated to a span when null terminator is at the end")
{
    BufferWrapper buffer{};

    const bool ok = microstrain::strings::concat_z(buffer.span, &buffer.index, CHECK_STRING);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer.array, CHECK_STRING);
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(CHECK_STRING_LENGTH, buffer.array);
}

TEST("Span string concatenation", "A C string is partially concatenated to a span when a max length is given")
{
    BufferWrapper buffer{};
    constexpr size_t character_limit = 4;

    const bool ok = microstrain::strings::concat_z(buffer.span, &buffer.index, "123456789", character_limit);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer.array, "1234");
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(character_limit, buffer.array);
}

TEST("Span string concatenation", "A string literal can be concatenated to a span")
{
    BufferWrapper buffer{};

    const bool ok = microstrain::strings::concat_l(buffer.span, &buffer.index, "123456789");

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer.array, "123456789");
    EXPECT_BUFFER_TO_BE_TERMINATED_AT_POSITION(9, buffer.array);
}

TEST("Span string formatting", "A span string can be formatted properly")
{
    BufferWrapper buffer{};

    const bool ok = microstrain::strings::format(buffer.span, &buffer.index, "%s %u %02X", "test", 100, 256);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer.array, "test 100 100");
}

TEST("Byte formatting", "A byte array can be formatted to a text buffer in hexadecimal")
{
    BufferWrapper buffer{};
    const uint8_t DATA[] = { 0x0F, 0x2E, 0x4D, 0x6C, 0x8B, 0xAA };

    const bool ok = microstrain::strings::bytesToHexStr(buffer.span, &buffer.index, DATA, 2);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer.array, "0F2E 4D6C 8BAA");
}
