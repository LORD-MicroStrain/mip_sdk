#include "microstrain_test.hpp"

#include <cstring>
#include <string>

namespace detail
{
#ifdef USE_DOCTEST
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

    void fail_if_position_out_of_bounds(const char *buffer, const size_t position)
    {
        const size_t buffer_size = strnlen(buffer, MAX_CHECK_STRING_LENGTH);

        if (position > buffer_size)
        {
            //INFO("Position can't be greater than the buffer size");
            INFO("Buffer size: " << buffer_size);
            INFO("Position:    " << position);
            FAIL("Position can't be greater than the buffer size");
        }
    }

	void check_buffer_terminated(const char *buffer, const size_t position)
    {
        fail_if_position_out_of_bounds(buffer, position);

        INFO("The buffer is not terminated");

        std::string last_five_characters = "Last five characters: ";
        last_five_characters += std::string(&buffer[position - 5], 6);
        INFO(last_five_characters);

        CHECK_EQ(buffer[position], '\0');
    }
#endif
}
