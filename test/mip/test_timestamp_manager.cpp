#include <array>
#include <limits>

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
        if (timestamp.getTimestamp() != Nanoseconds(value))
        {
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