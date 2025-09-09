#include "microstrain_test.hpp"

#include <cstring>
#include <string>

namespace detail
{
    void warn_c_strings_equal(const char* string1, const char* string2)
    {
        INFO(std::string(string1));
        INFO(std::string(string2));
        WARN_EQ(strcmp(string1, string2), 0);
    }

    void check_c_strings_equal(const char* string1, const char* string2)
    {
        INFO(std::string(string1));
        INFO(std::string(string2));
        CHECK_EQ(strcmp(string1, string2), 0);
    }

    void require_c_strings_equal(const char* string1, const char* string2)
    {
        INFO(std::string(string1));
        INFO(std::string(string2));
        REQUIRE_EQ(strcmp(string1, string2), 0);
    }
}