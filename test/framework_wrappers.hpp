#pragma once

#include <cstring>
#include <string>

#ifdef USE_DOCTEST
    #include <doctest/doctest.h>

    namespace detail
    {
        inline void warn_c_strings_equal(const char* string1, const char* string2)
        {
            INFO(std::string(string1));
            INFO(std::string(string2));
            WARN_EQ(strcmp(string1, string2), 0);
        }

        inline void check_c_strings_equal(const char* string1, const char* string2)
        {
            INFO(std::string(string1));
            INFO(std::string(string2));
            CHECK_EQ(strcmp(string1, string2), 0);
        }

        inline void require_c_strings_equal(const char* string1, const char* string2)
        {
            INFO(std::string(string1));
            INFO(std::string(string2));
            REQUIRE_EQ(strcmp(string1, string2), 0);
        }
    }

    #define WARN_IF_NOT_TRUE(condition) WARN(condition)
    #define EXPECT_TRUE(condition)      CHECK(condition)
    #define ASSERT_TRUE(condition)      REQUIRE(condition)

    #define WARN_IF_NOT_EQUAL(value1, value2) WARN_EQ(value1, value2)
    #define EXPECT_EQUAL(value1, value2)      CHECK_EQ(value1, value2)
    #define ASSERT_EQUAL(value1, value2)      REQUIRE_EQ(value1, value2)

    #define WARN_IF_C_STRINGS_NOT_EQUAL(value1, value2) detail::warn_c_strings_equal(value1, value2)
    #define EXPECT_C_STRINGS_EQUAL(value1, value2)      detail::check_c_strings_equal(value1, value2)
    #define ASSERT_C_STRINGS_EQUAL(value1, value2)      detail::require_c_strings_equal(value1, value2)
#endif