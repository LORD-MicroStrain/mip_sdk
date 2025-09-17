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

    void charEqual(const char actual, const char expected, FailureLevel failure_level)
    {
        std::ostringstream expected_output;
        std::ostringstream actual_output;
        expected_output << "Expected: " << getCharacterOutput(expected);
        actual_output   << "Actual:   " << getCharacterOutput(actual);

        INFO(expected_output.str());
        INFO(actual_output.str());

        switch(failure_level)
        {
            case FailureLevel::WARN:  WARN_EQ(actual, expected);    break;
            case FailureLevel::FAIL:  CHECK_EQ(actual, expected);   break;
            case FailureLevel::FATAL: REQUIRE_EQ(actual, expected); break;
        }
    }

    void cStringsEqual(const char* actual, const char* expected, const FailureLevel failure_level)
    {
        // It is reasonable to assume expected is zero-terminated as it is set by the test
        const size_t expected_size = strlen(expected);

        if (actual[expected_size] != '\0')
        {
            std::ostringstream expected_output;
            std::ostringstream actual_output;
            expected_output << "Expected: " << expected << getCharacterOutput(expected[expected_size]);
            actual_output   << "Actual:   " << actual;

            INFO("The actual string is not terminated when the expected string is:");
            INFO(expected_output.str());
            INFO(actual_output.str());

            switch(failure_level)
            {
                case FailureLevel::WARN:  WARN(false);    break;
                case FailureLevel::FAIL:  CHECK(false);   break;
                case FailureLevel::FATAL: REQUIRE(false); break;
            }
        }

        INFO("Actual:   " << std::string(actual, expected_size));
        INFO("Expected: " << std::string(expected));

        switch(failure_level)
        {
            case FailureLevel::WARN:  WARN_EQ(strncmp(actual, expected, expected_size), 0);    break;
            case FailureLevel::FAIL:  CHECK_EQ(strncmp(actual, expected, expected_size), 0);   break;
            case FailureLevel::FATAL: REQUIRE_EQ(strncmp(actual, expected, expected_size), 0); break;
        }
    }
}
