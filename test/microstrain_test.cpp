#include "microstrain_test.hpp"

#include <cstring>
#include <string>

namespace detail
{
#ifdef MICROSTRAIN_TEST_USE_DOCTEST
    void check_c_strings_equal(const char* actual, const char* expected)
    {
        const size_t safe_actual_size = strnlen(actual, MAX_STRING_LENGTH);
        const size_t safe_expected_size = strnlen(expected, MAX_STRING_LENGTH);

        INFO("Actual:   " << std::string(&actual[0], safe_actual_size));
        INFO("Expected: " << std::string(&expected[0], safe_expected_size));
        CHECK_EQ(strncmp(actual, expected, MAX_STRING_LENGTH), 0);
    }

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
