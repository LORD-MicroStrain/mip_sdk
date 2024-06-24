#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <typeinfo>

#include <mip/utils/timestamp_manager.hpp>

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

bool testGetTimestamp()
{
    static long long test_time = toNanoseconds(seconds_in_week) + toNanoseconds(500);

    for (auto &value : std::array<long long, 3>{min_nanoseconds, test_time, max_nanoseconds})
    {
        mip::TimestampManager timestamp(value);

        if (!getterTestCase("GetTimestamp-base", timestamp.getTimestamp(), 
            mip::Nanoseconds(value)))
        {
            return false;
        }
        
        if (!getterTestCase("GetTimestamp-template", timestamp.getTimestamp<mip::Seconds>(), 
            mip::Seconds(toSeconds(value))))
        {
            return false;            
        }
    }
    
    return true;
}

bool testGetTimeOfWeek() 
{
    static long long test_time = toNanoseconds(seconds_in_week) - toNanoseconds(500);
    static long long test_time2 = toNanoseconds(seconds_in_week) + toNanoseconds(500);

    for (auto &value : std::array<long long, 4>{min_nanoseconds, test_time, test_time2, max_nanoseconds})
    {
        mip::TimestampManager timestamp(value);
        long long time_of_week = value % nanoseconds_in_week;

        if (!getterTestCase("GetTimeOfWeek-base", timestamp.getTimeOfWeek(), 
            mip::Nanoseconds(time_of_week)))
        {
            return false;
        }

        if (!getterTestCase("GetTimeOfWeek-template", timestamp.getTimeOfWeek<mip::Seconds>(), 
            mip::Seconds(toSeconds(time_of_week))))
        {
            return false;
        }
    }
    
    return true;
}

int main(int argc, const char* argv[])
{
    static constexpr short success = 0;
    static constexpr short fail = 1;

    if (!testGetTimestamp() ) { return fail; }
    if (!testGetTimeOfWeek()) { return fail; }

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
