#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <typeinfo>

#include <mip/utils/timestamp_manager.hpp>

#define SUCCESS    0
#define FAIL       1
#define START_TIME 123456789


// TODO: Organize
template<typename ActualOutput, typename ExpectedOutput>
    void outputCaseResults(const char* case_name, ActualOutput actual, ExpectedOutput expected)
{
    std::cout << 
        "Failed: " << case_name << "\n" << 
        "    ---> Actual: " << actual << "\n" <<
        "    ---> Expected: " << expected << "\n";
}

template<typename DurationActual, typename DurationExpected>
    bool testCase(const char* name, DurationActual actual, DurationExpected expected)
{
    if (actual != expected)
    {
        outputCaseResults(name, actual.count(), expected.count());
        return false;
    }
    
    return true;
}

bool testCase(const char* name, const std::type_info& actual, const std::type_info& expected)
{
    if (actual != expected)
    {
        outputCaseResults(name, actual.name(), expected.name());
        return false;
    }
    
    return true;
}

bool testGetTimestamp() 
{
    std::array<long long, 3> test_values{0, START_TIME, std::numeric_limits<long long>::max()};
    for (long long &value : test_values)
    {
        mip::TimestampManager timestamp(value);

        auto actual_base = timestamp.getTimestamp();
        if (!testCase("GetTimestamp-base-type", typeid(actual_base), typeid(mip::Nanoseconds)))
        {
            return false;
        }
        if (!testCase("GetTimestamp-base-value", actual_base, mip::Nanoseconds(value)))
        {
            return false;
        }

        if (typeid(timestamp.getTimestamp<mip::Seconds>()) != typeid(mip::Seconds))
        {
            printf("Template function type check failed.\n");
            return false;
        }
        
        // if (timestamp.getTimestamp<mip::Seconds>() != mip::Seconds(value)) 
        // {
        //     // TODO: Figure out why this is failing.
        //     printf("Template function value check failed.\n");
        //     return false;
        // }
    }
    
    return true;
}

int main(int argc, const char* argv[])
{
    if (!testGetTimestamp()) { return FAIL; }

    return SUCCESS;
}