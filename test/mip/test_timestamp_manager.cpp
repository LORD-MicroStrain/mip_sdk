#include <mip/utils/timestamp_manager.hpp>

#define SUCCESS 0
#define    FAIL 1

bool testGetTimestampBase() 
{
    return true;    
}

bool testGetTimestampTemplate()
{
    return true;
}

int main(int argc, const char* argv[])
{
    if (    !testGetTimestampBase()) { return FAIL; }
    if (!testGetTimestampTemplate()) { return FAIL; }

    return SUCCESS;
}