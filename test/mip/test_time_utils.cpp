#include <array>
#include <iostream>

#include <mip/utils/timestamp.hpp>


// Time conversions
constexpr mip::Nanoseconds nanoseconds_in_second(1000000000);
constexpr mip::Seconds     seconds_in_week(604800);
constexpr mip::Nanoseconds nanoseconds_in_week((std::uint64_t)1000000000 * (std::uint64_t)604800);

// Test values
constexpr mip::Nanoseconds invalid_nanoseconds(-1);
constexpr mip::Seconds     invalid_seconds(-1);
constexpr mip::Nanoseconds main_test_nanoseconds(123456789);
constexpr mip::Seconds     main_test_seconds(123456789);

/** Mock objects ************************************************************************/

struct MockUnixTime : mip::UnixTime
{
    mip::Nanoseconds now() const override
    {
        return main_test_nanoseconds;
    }
};

/** Test case utilities *****************************************************************/

template<typename T1, typename T2>
bool getterTestCase(const std::string &name, T1 actual, T2 expected);

template<typename ExpectedException, typename Callable>
bool invalidInputTestCase(const char *name, Callable test_wrapper);

// Comprehensive failed message with actual vs. expected values.
template<typename T1, typename T2>
void outputCaseResults(const char* name, T1 actual, T2 expected);
// Simple failed message
void outputFailed(const char* name, const char* message);

/** Tests *******************************************************************************/

bool testManualConstructor()
{
    auto invalid_base = []() -> void { mip::TimestampExperimental(mip::UnixTime{}, invalid_nanoseconds); };
    if (!invalidInputTestCase<std::invalid_argument>("ManualConstructor-base", invalid_base))
    {
        return false;
    }

    auto invalid_template = []() -> void { mip::TimestampExperimental(mip::UnixTime{}, invalid_seconds); };
    if (!invalidInputTestCase<std::invalid_argument>("ManualConstructor-template", invalid_base))
    {
        return false;
    }
    
    return true;
}

bool testGetTimestamp()
{
    mip::TimestampExperimental timestamp_zero(mip::UnixTime{});
    mip::TimestampExperimental timestamp(mip::UnixTime{}, nanoseconds_in_second);

    if (!getterTestCase("GetTimestamp-zero", timestamp_zero.getTimestamp(), mip::Nanoseconds(0)))
    {
        return false;
    }
    
    if (!getterTestCase("GetTimestamp-base", timestamp.getTimestamp(), nanoseconds_in_second))
    {
        return false;
    }

    if (!getterTestCase("GetTimestamp-template", timestamp.getTimestamp<mip::Seconds>(), mip::Seconds(1)))
    {
        return false;
    }

    return true;
}

bool testSetTimestamp()
{
    mip::TimestampExperimental timestamp(mip::UnixTime{});

    auto invalid_base = [&timestamp]() -> void { timestamp.setTimestamp(invalid_nanoseconds); };
    if (!invalidInputTestCase<std::invalid_argument>("SetTimestampInvalid-base", invalid_base))
    {
        return false;
    }

    auto invalid_templated = [&timestamp]() -> void { timestamp.setTimestamp(invalid_seconds); };
    if (!invalidInputTestCase<std::invalid_argument>("SetTimestampInvalid-template", invalid_templated))
    {
        return false;
    }

    timestamp.setTimestamp(main_test_nanoseconds);
    if (!getterTestCase("SetTimestamp-base", timestamp.getTimestamp(), main_test_nanoseconds))
    {
        return false;
    }
    
    timestamp.setTimestamp(main_test_seconds);
    if (!getterTestCase("SetTimestamp-template", timestamp.getTimestamp<mip::Seconds>(), main_test_seconds))
    {
        return false;
    }

    return true;
}

bool testSynchronize()
{
    mip::TimestampExperimental timestamp(MockUnixTime{});

    timestamp.synchronize();
    if (!getterTestCase("Synchronize", timestamp.getTimestamp(), main_test_nanoseconds))
    {
        return false;
    }
    
    return true;
}

bool testNow()
{
    mip::TimestampExperimental timestamp = mip::TimestampExperimental::Now(MockUnixTime());
    
    if (!getterTestCase("Now", timestamp.getTimestamp(), main_test_nanoseconds))
    {
        return false;
    }

    return true;
}

    // static std::array<mip::Nanoseconds, 3> test_values{
    //     mip::Nanoseconds(0), 
    //     mip::Nanoseconds(nanoseconds_in_week - 500), 
    //     mip::Nanoseconds(nanoseconds_in_week + 500)
    // };

    //     if (!getterTestCase("GetTimeOfWeek-base", timestamp.getTimeOfWeek(), 
    //         mip::Nanoseconds(value % nanoseconds_in_week)))
    //     {
    //         return false;
    //     }

    //     if (!getterTestCase("GetTimeOfWeek-template", timestamp.getTimeOfWeek<mip::Seconds>(), 
    //         mip::Seconds(toSeconds(value) % seconds_in_week)))
    //     {
    //         return false;
    //     }
    // }
// }

int main(int argc, const char* argv[])
{
    static constexpr short success = 0;
    static constexpr short fail = 1;

    if (!testManualConstructor() ||
        !testGetTimestamp()      ||
        !testSetTimestamp()      ||
        !testSynchronize()       ||
        !testNow()                )
    {
        return fail;
    }

    return success;
}


/**************************************************************************************/
/* NOTE: The following are definitions for all declarations above. There are no new   */
/*       declarations following this statement.                                       */
/**************************************************************************************/

template<typename T1, typename T2>
bool getterTestCase(const std::string &name, T1 actual, T2 expected)
{
    const std::type_info& actual_type = typeid(actual);
    const std::type_info& expected_type = typeid(expected);
    
    if (actual_type != expected_type)
    {
        outputCaseResults((name + "-type").c_str(), actual_type.name(), expected_type.name());
        return false;
    }

    if (actual != expected)
    {
        outputCaseResults((name + "-value").c_str(), actual.count(), expected.count());
        return false;
    }
    
    return true;
}

template<typename ExpectedException, typename Callable>
bool invalidInputTestCase(const char *name, Callable test_wrapper)
{
    try
    {
        test_wrapper();
    }
    catch(const ExpectedException&)
    {
        return true;
    }
    
    outputFailed(name, "No exception was raised for invalid input.");
    return false;
}

template<typename T1, typename T2>
void outputCaseResults(const char* name, T1 actual, T2 expected)
{
    std::cout << 
        "Failed: " << name << "\n" << 
        "    --->   Actual: " << actual << "\n" <<
        "    ---> Expected: " << expected << "\n";
}

void outputFailed(const char* name, const char* message)
{
    std::cout <<
        "Failed: " << name << "\n" <<
        "   ---> " << message << "\n";
}
