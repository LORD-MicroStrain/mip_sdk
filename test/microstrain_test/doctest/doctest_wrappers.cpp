#include "doctest_wrappers.hpp"

#include <cstring>
#include <sstream>
#include <string>

namespace detail
{
    // TODO: Can probably move this to a general utility. Could be useful for other things.
    /// @brief Formats any character for printing.
    ///
    /// This is used to guarantee that special characters can be displayed.
    std::string formatCharacterOutput(const char c)
    {
        switch(c)
        {
            case '\0': return "\\0";
            default:   return std::string{c};
        }
    }

    /// @brief Equality check for two single characters (char 1 == char 2).
    ///
    /// Output for test failure will display both characters, even if they are special characters.
    void charEqual(const char actual, const char expected, FailureLevel failure_level)
    {
        std::ostringstream expected_output;
        std::ostringstream actual_output;
        expected_output << "Expected: " << formatCharacterOutput(expected);
        actual_output   << "Actual:   " << formatCharacterOutput(actual);

        INFO(expected_output.str());
        INFO(actual_output.str());

        switch(failure_level)
        {
            case FailureLevel::WARN:  WARN_EQ(actual, expected);    break;
            case FailureLevel::FAIL:  CHECK_EQ(actual, expected);   break;
            case FailureLevel::FATAL: REQUIRE_EQ(actual, expected); break;
        }
    }

    /// @brief Safe equality check for two C strings.
    ///
    /// Output for test failure will display both strings.
    ///
    /// The expected string must be zero-terminated, but the actual string does not necessarily need to be. The order of
    /// arguments matters in this case, as the expected string is used to calculate the max size for safety. If the
    /// actual string is not zero-terminated, a separate test failure action at the given failure level will be evoked.
    void cStringsEqual(const char* actual, const char* expected, const FailureLevel failure_level)
    {
        // It is reasonable to assume expected is zero-terminated as it is set by the test
        const size_t expected_size = strlen(expected);

        if (actual[expected_size] != '\0')
        {
            std::ostringstream expected_output;
            std::ostringstream actual_output;
            expected_output << "Expected: " << expected << formatCharacterOutput(expected[expected_size]);
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
