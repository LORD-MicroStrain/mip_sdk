#include <array>
#include <iostream>

#include <mip/utils/timestamp.hpp>

constexpr long long min_nanoseconds = 0;
constexpr long long max_nanoseconds = std::numeric_limits<long long>::max();
constexpr long long nanoseconds_in_second = 1000000000; 
constexpr long long seconds_in_week = 604800;
constexpr long long nanoseconds_in_week = nanoseconds_in_second * seconds_in_week;


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
    auto invalid_base = []() -> void { mip::TimestampExperimental(mip::UnixTime{}, mip::Nanoseconds(-1)); };
    if (!invalidInputTestCase<std::invalid_argument>("ManualConstructor-base", invalid_base))
    {
        return false;
    }

    auto invalid_template = []() -> void { mip::TimestampExperimental(mip::UnixTime{}, mip::Seconds(-1)); };
    if (!invalidInputTestCase<std::invalid_argument>("ManualConstructor-template", invalid_base))
    {
        return false;
    }
    
    return true;
}

bool testGetTimestamp()
{
    mip::TimestampExperimental timestamp_zero(mip::UnixTime{});
    if (!getterTestCase("GetTimestamp-zero", timestamp_zero.getTimestamp(), mip::Nanoseconds(0)))
    {
        return false;
    }
    
    mip::Nanoseconds expected_base(nanoseconds_in_second);
    mip::TimestampExperimental timestamp(mip::UnixTime{}, expected_base);
    if (!getterTestCase("GetTimestamp-base", timestamp.getTimestamp(), expected_base))
    {
        return false;
    }

    mip::Seconds expected_template(2);
    if (!getterTestCase("GetTimestamp-template", timestamp.getTimestamp<mip::Seconds>(), expected_template))
    {
        return false;
    }

    return true;
}

bool testSetTimestamp()
{
    mip::TimestampExperimental timestamp(mip::UnixTime{});

    auto invalid_base = [&timestamp]() -> void { timestamp.setTimestamp(mip::Nanoseconds(-1)); };
    if (!invalidInputTestCase<std::invalid_argument>("SetTimestampInvalid-base", invalid_base))
    {
        return false;
    }

    auto invalid_templated = [&timestamp]() -> void { timestamp.setTimestamp(mip::Seconds(-1)); };
    if (!invalidInputTestCase<std::invalid_argument>("SetTimestampInvalid-template", invalid_templated))
    {
        return false;
    }

    mip::Nanoseconds expected_base(123456789);
    timestamp.setTimestamp(expected_base);
    if (!getterTestCase("SetTimestamp-base", timestamp.getTimestamp(), expected_base))
    {
        return false;
    }
    
    mip::Seconds expected_templated(123456789);
    timestamp.setTimestamp(expected_templated);
    if (!getterTestCase("SetTimestamp-template", timestamp.getTimestamp<mip::Seconds>(), expected_templated))
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

static constexpr mip::Nanoseconds mock_sync_time(123456789);

struct MockUnixTime : mip::UnixTime
{
    mip::Nanoseconds now() const override
    {
        return mock_sync_time;
    }
};

bool testSynchronize()
{
    mip::TimestampExperimental timestamp(MockUnixTime{});
    timestamp.synchronize();

    if (!getterTestCase("Synchronize", timestamp.getTimestamp(), mock_sync_time))
    {
        return false;
    }
    
    return true;
}

bool testNow()
{
    mip::TimestampExperimental timestamp = mip::TimestampExperimental::Now(MockUnixTime());
    
    mip::Nanoseconds actual = timestamp.getTimestamp();
    mip::Nanoseconds expected = mock_sync_time;
    if (!getterTestCase("Now", actual, expected))
    {
        return false;
    }

    return true;
}

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
