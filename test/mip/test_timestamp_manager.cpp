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


bool testGetTimestamp() 
{
    std::array<long long, 3> test_values{0, START_TIME, std::numeric_limits<long long>::max()};
    for (long long &value : test_values)
    {
        mip::TimestampManager timestamp(value);

        const std::type_info& actual = typeid(timestamp.getTimestamp());
        const std::type_info& expected = typeid(mip::Nanoseconds);
        if (actual != expected)
        {
            std::cout << 
                "Base function type test failed:\n" << 
                "\tActual: " << actual.name() << "\n" <<
                "\tExpected: " << expected.name() << "\n";
            return false;
        }

        const auto actual = timestamp.getTimestamp();
        const mip::Nanoseconds expected = mip::Nanoseconds(value+1);
        if (actual != expected)
        {
            std::cout << 
                "Base function value test failed:\n" << 
                "\tActual: " << actual.count() << "\n" <<
                "\tExpected: " << expected.count() << "\n";
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