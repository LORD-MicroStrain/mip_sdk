#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <typeinfo>

#include <mip/utils/timestamp_manager.hpp>

constexpr short success = 0;
constexpr short fail = 1;
constexpr long long nanoseconds_in_second = 1000000000; 
constexpr int seconds_in_week = 604800;
constexpr int weeks_in_year = 52;


/** Misc. Utilities *********************************************************************/

mip::Seconds toSeconds(long long nanoseconds);

/** Test case utilities *****************************************************************/

// TODO: Change to getterTestCase.
template<typename DurationActual, typename DurationExpected>
    bool testCase(std::string name, DurationActual actual, DurationExpected expected)
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

bool testGetters() 
{
    constexpr long long test_time = (long long)seconds_in_week * nanoseconds_in_second + (500 * nanoseconds_in_second);

    // Edge values
    for (long long &value : std::array<long long, 3>{0, test_time, std::numeric_limits<long long>::max()})
    {
        mip::TimestampManager timestamp(value);

        if (!testCase("GetTimestamp-base", timestamp.getTimestamp(), mip::Nanoseconds(value)))
        {
            return false;
        }
        
        if (!testCase("GetTimestamp-template", timestamp.getTimestamp<mip::Seconds>(), toSeconds(value)))
        {
            return false;            
        }

        // TODO: Update
        if (!testCase("GetTimeOfWeek-base", timestamp.getTimeOfWeek(), mip::Nanoseconds(value % test_time)))
        {
            return false;
        }

        if (!testCase("GetTimeOfWeek-template", timestamp.getTimeOfWeek<mip::Seconds>(), toSeconds(value % test_time)))
        {
            return false;
        }
    }
    
    return true;
}

int main(int argc, const char* argv[])
{
    if (!testGetters()) 
    { 
        return fail; 
    }

    return success;
}


/**************************************************************************************/
/* NOTE: The following are definitions for all utility declarations above. There are  */
/*       no new declarations following this statement.                                */
/**************************************************************************************/

mip::Seconds toSeconds(long long nanoseconds)
{
    return mip::Seconds(nanoseconds / nanoseconds_in_second);
}
