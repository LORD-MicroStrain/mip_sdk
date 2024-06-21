#include <array>
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
        if (typeid(timestamp.getTimestamp()) != typeid(Nanoseconds))
        {
            printf("Base function type check failed.\n");
            return false;
        }

        if (timestamp.getTimestamp() != Nanoseconds(value))
        {
            printf("Base function value check failed.\n");
            return false;
        }

        if (typeid(timestamp.getTimestamp<Seconds>()) != typeid(Seconds))
        {
            printf("Template function type check failed.\n");
            return false;
        }
        
        if (timestamp.getTimestamp<Seconds>() != Seconds(value)) 
        {
            // TODO: Figure out why this is failing.
            printf("Template function value check failed.\n");
            return false;
        }
    }
    
    return true;
}

int main(int argc, const char* argv[])
{
    if (!testGetTimestamp()) { return FAIL; }

    return SUCCESS;
}