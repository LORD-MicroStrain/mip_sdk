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


/** Misc. Utilities *********************************************************************/

long long toNanoseconds(int seconds);
long long toSeconds(long long nanoseconds);

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
        "    ---> Actual: " << actual << "\n" <<
        "    ---> Expected: " << expected << "\n";
}

/** Tests *******************************************************************************/

bool testInvalidConstructors()
{
    mip::Nanoseconds zero(0);
    try 
    {
        mip::TimestampExperimental(mip::UnixTime(), zero);
    }
    catch (const std::invalid_argument& e)
    {
        return true;
    }

    return false;
}

bool testGetTimestampBase()
{
    // mip::Nanoseconds less_than_week = mip::Nanoseconds(nanoseconds_in_week - 500);
    // mip::Nanoseconds greater_than_week = mip::Nanoseconds(nanoseconds_in_week + 500);
    
    // for (auto &test_value : std::array<mip::Nanoseconds, 2>{less_than_week, greater_than_week})
    // {
    //    mip::TimestampExperimental(mip::UnixTime(), test_value); 
       
    //     if (!getterTestCase("GetTimestamp-base", timestamp.getTimestamp(), 
    //         mip::Nanoseconds(value)))
    //     {
    //         return false;
    //     }
    // }
    
    return true;
}
    // static long long test_time = toNanoseconds(seconds_in_week) - toNanoseconds(500);
    // static long long test_time2 = toNanoseconds(seconds_in_week) + toNanoseconds(500);

    // for (auto &value : std::array<long long, 4>{min_nanoseconds, test_time, test_time2, max_nanoseconds})
    // {
    //     mip::TimestampExperimental timestamp(value);

    //     if (!getterTestCase("GetTimestamp-base", timestamp.getTimestamp(), 
    //         mip::Nanoseconds(value)))
    //     {
    //         return false;
    //     }
        
    //     if (!getterTestCase("GetTimestamp-template", timestamp.getTimestamp<mip::Seconds>(), 
    //         mip::Seconds(toSeconds(value))))
    //     {
    //         return false;            
    //     }

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
bool testSynchronize()
{
    // TODO: Rewrite these with new structure.
    // TODO: Add check for constructor value using Unix time when implemented.

    // std::array<mip::TimeStandard, 1> standards{
    //     mip::TimeStandard::GPS
    // };

    // for (mip::TimeStandard& standard : standards)
    // {
    //     mip::TimestampManager timestamp(standard);
    //     mip::Nanoseconds synced_time = timestamp.getTimestamp();

    //     if (synced_time == mip::Nanoseconds(0))
    //     {
    //         outputCaseResults("SyncConstructor-GPS", synced_time.count(), "time since epoch >= 0");
    //         return false;
    //     }
        
    //     static constexpr short leap = 18; 
    //     static constexpr int epoch_gap = 315964800; 
    // }

    return true;
}

int main(int argc, const char* argv[])
{
    static constexpr short success = 0;
    static constexpr short fail = 1;

    if (!testInvalidConstructors())
    {
        return fail;
    }
    if (!testGetTimestampBase()) 
    { 
        return fail; 
    }
    if (!testSynchronize())
    {
        return fail;
    }

    return success;
}


/**************************************************************************************/
/* NOTE: The following are definitions for all utility declarations above. There are  */
/*       no new declarations following this statement.                                */
/**************************************************************************************/

long long toNanoseconds(int seconds)
{
    return seconds * nanoseconds_in_second;
}

long long toSeconds(long long nanoseconds)
{
    return nanoseconds / nanoseconds_in_second;
}
