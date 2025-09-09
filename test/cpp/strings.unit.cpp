#define MICROSTRAIN_HAS_STD_STRING 1

#include <string_view>

#include <framework_wrappers.hpp>
#include <microstrain/strings.hpp>

static constexpr char TEST_STRING[] = "Testing!";

struct Buffer
{
    char array[1024] = {};
    microstrain::Span<char> span{array};
    size_t index = 0;
};

// TODO: Figure out naming for these tests


TEST("Span string concatenation", "A C string can be concatenated to an explicitly created span")
{
    Buffer buffer{};

    const bool ok = microstrain::strings::concat(buffer.span, &buffer.index, TEST_STRING, sizeof(TEST_STRING) - 1);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, TEST_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A C string can be concatenated to an implicitly created span")
{
    Buffer buffer{};

    const bool ok = microstrain::strings::concat({buffer.array, sizeof(buffer.array)}, &buffer.index, TEST_STRING, sizeof(TEST_STRING)-1);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, TEST_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A span can be concatenated to another span")
{
    Buffer buffer{};

    const bool ok = microstrain::strings::concat(buffer.span, &buffer.index, microstrain::Span<const char>{TEST_STRING});

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, TEST_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A string view can be concatenated to a span")
{
    Buffer buffer{};

    const bool ok = microstrain::strings::concat(buffer.span, &buffer.index, std::string_view{TEST_STRING});

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, TEST_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A string can be concatenated to a span")
{
    Buffer buffer{};

    const bool ok = microstrain::strings::concat(buffer.span, &buffer.index, std::string{TEST_STRING});

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, TEST_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A C string can be fully concatenated to a span when null terminator is at the end")
{
    Buffer buffer{};

    const bool ok = microstrain::strings::concat_z(buffer.span, &buffer.index, TEST_STRING);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, TEST_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string concatenation", "A C string is partially concatenated to a span when a max length is given")
{
    Buffer buffer{};
    constexpr size_t character_limit = 4;

    const bool ok = microstrain::strings::concat_z(buffer.span, &buffer.index, TEST_STRING, character_limit);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, "Test");
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), num_chars, "");
}

TEST("Span string concatenation", "A string literal can be concatenated to a span")
{
    Buffer buffer{};

    const bool ok = microstrain::strings::concat_l(buffer.span, &buffer.index, "Testing!");

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, TEST_STRING);
    //TEST_ASSERT_BUFFER_TERMINATED(test.array, sizeof(test.array), sizeof(FAKE_STRING)-1, "");
}

TEST("Span string formatting", "An span string can be formatted properly")
{
    Buffer buffer{};

    const bool ok = microstrain::strings::format(buffer.span, &buffer.index, "%s %u %02X", "test", 100, 256);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, "test 100 100");
}

TEST("Byte formatting", "A byte array can be formatted to a text buffer in hexadecimal")
{
    Buffer buffer{};
    const uint8_t DATA[] = { 0x0F, 0x2E, 0x4D, 0x6C, 0x8B, 0xAA };

    const bool ok = microstrain::strings::bytesToHexStr(buffer.span, &buffer.index, DATA, 2);

    EXPECT_TRUE(ok);
    EXPECT_C_STRINGS_EQUAL(buffer.array, "0F2E 4D6C 8BAA");
}
