#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <typeinfo>

#include <mip/utils/timestamp.hpp>

constexpr long long min_nanoseconds = 0;
constexpr long long max_nanoseconds = std::numeric_limits<long long>::max();
constexpr long long nanoseconds_in_second = 1000000000; 
constexpr long long seconds_in_week = 604800;
constexpr long long nanoseconds_in_week = nanoseconds_in_second * seconds_in_week;


/** Test case utilities *****************************************************************/

template<typename DurationActual, typename DurationExpected>
    bool getterTestCase(std::string name, DurationActual actual, DurationExpected expected)
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

template<typename ActualOutput, typename ExpectedOutput>
    void outputCaseResults(const char* case_name, ActualOutput actual, ExpectedOutput expected)
{
    std::cout << 
        "Failed: " << case_name << "\n" << 
        "    --->   Actual: " << actual << "\n" <<
        "    ---> Expected: " << expected << "\n";
}

void outputFailed(const char* case_name, const char* message)
{
    std::cout <<
        "Failed: " << case_name << "\n" <<
        "   ---> " << message << "\n";
}

/** Tests *******************************************************************************/

bool testManualTimeConstructorInvalid();
bool testManualTimeConstructorValid();
bool testGetTimestamp();
bool testSynchronize();
bool testNow();

int main(int argc, const char* argv[])
{
    static constexpr short success = 0;
    static constexpr short fail = 1;

    if (!testManualTimeConstructorInvalid() || !testManualTimeConstructorValid())
    {
        return fail;
    }
    if (!testGetTimestamp())
    { 
        return fail; 
    }
    if (!testSynchronize() || !testNow())
    {
        return fail;
    }

    return success;
}

bool testManualTimeConstructorInvalid()
{
    mip::Nanoseconds negative(-1);

    try 
    {
        mip::TimestampExperimental(mip::UnixTime(), negative);
    }
    catch (const std::invalid_argument&)
    {
        return true;
    }

    outputFailed("ManualConstructor", "invalid_argument not raised when time < 0");
    return false;
}

bool testManualTimeConstructorValid()
{
    mip::Nanoseconds base(1);
    mip::Seconds templated(1);
    
    try
    {
        mip::TimestampExperimental(mip::UnixTime{});
        mip::TimestampExperimental(mip::UnixTime{}, base);
        mip::TimestampExperimental(mip::UnixTime{}, templated);
    }
    catch(const std::invalid_argument&)
    {
        outputFailed("ManualConstructor", "invalid_argument raised when inputs are valid");
        return false;
    }

    return true; 
}

bool testGetTimestamp()
{
    std::array<mip::Nanoseconds, 3> test_values{
        mip::Nanoseconds(0), 
        mip::Nanoseconds(nanoseconds_in_week - 500), 
        mip::Nanoseconds(nanoseconds_in_week + 500)
    };
    
    for (auto &value : test_values)
    {
        mip::TimestampExperimental timestamp(mip::UnixTime(), value); 
       
        auto base_actual = timestamp.getTimestamp();
        if (!getterTestCase("GetTimestamp-base", timestamp.getTimestamp(), value))
        {
            return false;
        }

        auto template_actual = timestamp.getTimestamp<mip::Seconds>();
        mip::Seconds template_expected = std::chrono::duration_cast<mip::Seconds>(value);
        if (!getterTestCase("GetTimestamp-template", template_actual, template_expected))
        {
            return false;
        }
    }
    
    return true;
}

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

    mip::Nanoseconds actual = timestamp.getTimestamp();
    mip::Nanoseconds expected = mock_sync_time;
    if (!getterTestCase("Synchronize", actual, expected))
    {
        return false;
    }
    
    return true;
}

bool testNow()
{
    auto timestamp = mip::TimestampExperimental::Now(MockUnixTime());
    
    mip::Nanoseconds actual = timestamp.getTimestamp();
    mip::Nanoseconds expected = mock_sync_time;
    if (!getterTestCase("Now", actual, expected))
    {
        return false;
    }

    return true;
}
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
