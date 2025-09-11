#include "microstrain_test.hpp"

#include <cstring>
#include <string>

namespace detail
{
#ifdef MICROSTRAIN_TEST_USE_DOCTEST
    void check_c_strings_equal(const char* actual, const char* expected)
    {
        // Expected should be set by test and zero-terminated
        const size_t expected_size = strlen(expected);

        if (actual[expected_size] != '\0')
        {
            INFO("Got at position instead: " << std::string(1, actual[expected_size]));
            FAIL("The actual string is not terminated at the same position as the expected string");
        }

        INFO("Actual:   " << std::string(&actual[0], expected_size));
        INFO("Expected: " << std::string(&expected[0], expected_size));
        CHECK_EQ(strncmp(actual, expected, expected_size), 0);
    }

    static constexpr size_t MAX_STRING_LENGTH = 1024;


    void fail_if_position_out_of_bounds(const char *buffer, const size_t position)
    {
        const size_t buffer_size = strnlen(buffer, MAX_STRING_LENGTH);

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
