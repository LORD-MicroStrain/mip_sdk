#include "doctest_wrappers.hpp"

#include <cstring>
#include <sstream>
#include <string>

namespace detail
{
    void verifyCStringsAreEqual(const char* actual, const char* expected, const FAILURE_LEVEL level)
    {
        // It is reasonable to assume expected is zero-terminated as it is set by the test
        const size_t expected_size = strlen(expected);

        if (actual[expected_size] != '\0')
        {
            INFO("Got at position instead: " << std::string(1, actual[expected_size]));
            FAIL_CHECK("The actual string is not terminated at the same position as the expected string");
        }

        INFO("Actual:   " << std::string(&actual[0], expected_size));
        INFO("Expected: " << std::string(&expected[0], expected_size));

        if (level == FAILURE_LEVEL::WARN)
        {
            WARN_EQ(strncmp(actual, expected, expected_size), 0);
        }
        else if (level == FAILURE_LEVEL::FAIL)
        {
            CHECK_EQ(strncmp(actual, expected, expected_size), 0);
        }
        else if (level == FAILURE_LEVEL::EXIT)
        {
            REQUIRE_EQ(strncmp(actual, expected, expected_size), 0);
        }
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

    void verifyBufferTerminated(const char *buffer, const size_t buffer_size, const size_t position, FAILURE_LEVEL level)
    {
        fail_if_position_out_of_bounds(buffer_size, position);

        INFO("The buffer is not terminated at position: " << position);
        const size_t start = std::max(static_cast<size_t>(0), position - 4);
        const size_t length = position - start + 1;
        INFO("Last max(5, N) characters: " << std::string(&buffer[start], length));

        if (level == FAILURE_LEVEL::WARN)
        {
            WARN_EQ(buffer[position], '\0');
        }
        else if (level == FAILURE_LEVEL::FAIL)
        {
            CHECK_EQ(buffer[position], '\0');
        }
        else if (level == FAILURE_LEVEL::EXIT)
        {
            REQUIRE_EQ(buffer[position], '\0');
        }
    }
}