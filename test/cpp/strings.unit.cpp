#define MICROSTRAIN_HAS_STD_STRING 1

#include <string_view>

#include <microstrain_test.hpp>
#include <microstrain/strings.hpp>

static constexpr char CHECK_STRING[] = "Test: \"quotes\" 'single' & <xml/> {json} [array] $100 €50 ¥200 @user #tag 🚀 \n\t\\ 100% café naïve";
static constexpr size_t CHECK_STRING_LENGTH = sizeof(CHECK_STRING) - 1; // Length without null terminator

TEST("C++ string concatenation", "A char array view can be concatenated to a buffer")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer, &index, microstrain::ConstCharArrayView{CHECK_STRING});

    FAIL_IF_NOT_TRUE(!ok); // TODO: Change back
    FAIL_IF_C_STRINGS_NOT_EQUAL(buffer, CHECK_STRING);
    FAIL_IF_CHAR_NOT_EQUAL(buffer[CHECK_STRING_LENGTH], '\0');
}

TEST("C++ string concatenation", "A std::string_view can be concatenated to a buffer")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer, &index, std::string_view{CHECK_STRING});

    FAIL_IF_NOT_TRUE(ok);
    FAIL_IF_C_STRINGS_NOT_EQUAL(buffer, CHECK_STRING);
    FAIL_IF_CHAR_NOT_EQUAL(buffer[CHECK_STRING_LENGTH], '\0');
}

TEST("C++ string concatenation", "A std::string can be concatenated to a buffer")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat(buffer, &index, std::string{CHECK_STRING});

    FAIL_IF_NOT_TRUE(ok);
    FAIL_IF_C_STRINGS_NOT_EQUAL(buffer, CHECK_STRING);
    FAIL_IF_CHAR_NOT_EQUAL(buffer[CHECK_STRING_LENGTH], '\0');
}

TEST("C++ string concatenation", "A zero-terminated C string can be concatenated to a buffer")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat_cstr(buffer, &index, CHECK_STRING);

    FAIL_IF_NOT_TRUE(ok);
    FAIL_IF_C_STRINGS_NOT_EQUAL(buffer, CHECK_STRING);
    FAIL_IF_CHAR_NOT_EQUAL(buffer[CHECK_STRING_LENGTH], '\0');
}

TEST("C++ string concatenation", "Up to N characters of a zero-terminated string can be concatenated to a buffer")
{
    char buffer[1024] = {};
    size_t index = 0;
    constexpr size_t character_limit = 4;

    const bool ok = microstrain::strings::concat_cstr(buffer, &index, "123456789", character_limit);

    FAIL_IF_NOT_TRUE(ok);
    FAIL_IF_C_STRINGS_NOT_EQUAL(buffer, "1234");
    FAIL_IF_CHAR_NOT_EQUAL(buffer[character_limit], '\0');
}

TEST("C++ string concatenation", "A string literal can be concatenated to a buffer")
{
    char buffer[1024] = {};
    size_t index = 0;

    const bool ok = microstrain::strings::concat_l(buffer, &index, "123456789");

    FAIL_IF_NOT_TRUE(ok);
    FAIL_IF_C_STRINGS_NOT_EQUAL(buffer, "123456789");
    FAIL_IF_CHAR_NOT_EQUAL(buffer[9], '\0');
}
