#include "doctest_wrappers.hpp"

#include <cstring>
#include <sstream>
#include <string>

namespace detail
{
    std::string getCharacterOutput(const char c)
    {
        switch(c)
        {
            case '\0': return "\\0";
            default:   return std::string{c};
        }
    }

    void charsEqual(const char actual, const char expected)
    {
        std::ostringstream expected_output;
        std::ostringstream actual_output;
        expected_output << "Expected: " << getCharacterOutput(expected);
        actual_output   << "Actual:   " << getCharacterOutput(actual);

        INFO(expected_output.str());
        INFO(actual_output.str());

        CHECK_EQ(actual, expected);
    }

    void verifyCStringsAreEqual(const char* actual, const char* expected, const FAILURE_LEVEL level)
    {
        // It is reasonable to assume expected is zero-terminated as it is set by the test
        const size_t expected_size = strlen(expected);

        if (actual[expected_size] != '\0')
        {
            FAIL(
                "The actual string is not terminated when the expected string is:\n"
                "Actual (up to expected + 1): " << std::string(actual, expected_size + 1) << "\n"
                "Expected                   : " << std::string(expected) << "\n"
            );
        }

        INFO("Actual:   " << std::string(actual, expected_size));
        INFO("Expected: " << std::string(expected));

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
            FAIL(
                "Position out of bounds (can't be greater than the buffer size):\n"
                "Buffer size: " << buffer_size << "\n"
                "Position:    " << position << "\n";
            );
        }
    }

    void verifyBufferTerminated(const char *buffer, const size_t buffer_size, const size_t position, FAILURE_LEVEL level)
    {
        fail_if_position_out_of_bounds(buffer_size, position);

        INFO("The buffer is not terminated at position: " << position);
        const size_t start = (position < 4) ? 0 : position - 4;
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