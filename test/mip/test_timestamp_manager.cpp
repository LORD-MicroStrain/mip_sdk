#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <typeinfo>

#include <mip/utils/timestamp_manager.hpp>

constexpr short success = 0;
constexpr short fail = 1;
constexpr long long start_time = 9000000000; // 9 seconds (in nanoseconds)


template<typename DurationActual, typename DurationExpected>
    bool testCase(std::string name, DurationActual actual, DurationExpected expected);

template<typename ActualOutput, typename ExpectedOutput>
    void outputCaseResults(const char* case_name, ActualOutput actual, ExpectedOutput expected);

mip::Seconds toSeconds(long long nanoseconds);

bool testGetTimestamp() 
{
    std::array<long long, 3> test_values{0, start_time, std::numeric_limits<long long>::max()};
    for (long long &value : test_values)
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
    }
    
    return true;
}

int main(int argc, const char* argv[])
{
    if (!testGetTimestamp()) { return fail; }

    return success;
}


/**************************************************************************************/
/* NOTE: The following are definitions for all utility declarations above. There are  */
/*       no new declarations following this statement.                                */
/**************************************************************************************/

template<typename DurationActual, typename DurationExpected>
    inline bool testCase(std::string name, DurationActual actual, DurationExpected expected)
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
    inline void outputCaseResults(const char* case_name, ActualOutput actual, ExpectedOutput expected)
{
    std::cout << 
        "Failed: " << case_name << "\n" << 
        "    ---> Actual: " << actual << "\n" <<
        "    ---> Expected: " << expected << "\n";
}

mip::Seconds toSeconds(long long nanoseconds)
{
    return mip::Seconds(nanoseconds / 1000000000);
}
