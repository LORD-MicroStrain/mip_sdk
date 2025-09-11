#include "microstrain_test.hpp"

#include <cstring>
#include <sstream>
#include <string>

#ifdef MICROSTRAIN_TEST_USE_DOCTEST
namespace detail
{
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

    void fail_if_position_out_of_bounds(const size_t buffer_size, const size_t position)
    {
        if (position > buffer_size)
        {
            INFO("Position can't be greater than the buffer size");
            INFO("Buffer size: " << buffer_size);
            INFO("Position:    " << position);
            FAIL("Position out of bounds");
        }
    }

	void check_buffer_terminated(const char *buffer, const size_t buffer_size, const size_t position)
    {
        fail_if_position_out_of_bounds(buffer_size, position);

        INFO("The buffer is not terminated at position: " << position);
        const size_t start = std::max(static_cast<size_t>(0), position - 4);
        const size_t length = position - start + 1;
        INFO("Last max(5, N) characters: " << std::string(&buffer[start], length));

        CHECK_EQ(buffer[position], '\0');
    }
}
#endif
