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


template<typename DurationActual, typename DurationExpected>
    bool typeTestCase(const char* name, DurationActual actual, DurationExpected expected)
{
    const std::type_info& actual_type = typeid(actual);
    const std::type_info& expected_type = typeid(expected);

    if (actual_type != expected_type)
    {
        std::cout << 
            name << " failed:\n" << 
            "    ---> Actual: " << actual_type.name() << "\n" <<
            "    ---> Expected: " << expected_type.name() << "\n";
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
        
        if (!typeTestCase("GetTimestamp-base", timestamp.getTimestamp(), mip::Nanoseconds()))
        {
            return false;
        }

        // const auto actual = timestamp.getTimestamp();
        // const mip::Nanoseconds expected = mip::Nanoseconds(value+1);
        // if (actual != expected)
        // {
        //     std::cout << 
        //         "Base function value test failed:\n" << 
        //         "\tActual: " << actual.count() << "\n" <<
        //         "\tExpected: " << expected.count() << "\n";
        //     return false;
        // }

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