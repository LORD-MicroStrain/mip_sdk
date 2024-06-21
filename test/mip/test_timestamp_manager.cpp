#include <mip/utils/timestamp_manager.hpp>

#define SUCCESS    0
#define FAIL       1
#define START_TIME 123456789

mip::TimestampManager setUp()
{
    return mip::TimestampManager(START_TIME);
}

bool testGetTimestampBase() 
{
    mip::TimestampManager timestamp = setUp();
    Nanoseconds raw_timestamp = timestamp.getTimestamp();
    if (timestamp.getTimestamp() != Nanoseconds(START_TIME))
    {
        return false;
    }

    return true;    
}

bool testGetTimestampTemplate()
{
    return true;
}

int main(int argc, const char* argv[])
{
    if (!testGetTimestampBase()    ) { return FAIL; }
    if (!testGetTimestampTemplate()) { return FAIL; }

    return SUCCESS;
}