#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <typeinfo>

#include <mip/utils/timestamp.hpp>

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

void outputFailed(const char* case_name, const char* message)
{
    std::cout <<
        "Failed: " << case_name << "\n" <<
        "   ---> " << message << "\n";
}

/** Tests *******************************************************************************/

bool testManualTimeConstructorInvalid();
bool testManualTimeConstructorValid();
bool testGetTimestamp();
bool testSynchronizeUnix();
bool testSynchronizeStandards();
bool testNow();

int main(int argc, const char* argv[])
{
    static constexpr short success = 0;
    static constexpr short fail = 1;

    if (!testManualTimeConstructorInvalid() || !testManualTimeConstructorValid())
    {
        return fail;
    }
    if (!testGetTimestamp())
    { 
        return fail; 
    }
    if (!testSynchronizeUnix() || !testSynchronizeStandards() || !testNow())
    {
        return fail;
    }

    return success;
}

bool testManualTimeConstructorInvalid()
{
    mip::Nanoseconds negative(-1);

    try 
    {
        mip::TimestampExperimental(mip::UnixTime(), negative);
    }
    catch (const std::invalid_argument&)
    {
        return true;
    }

    outputFailed("ManualConstructor", "invalid_argument not raised when time < 0");
    return false;
}

bool testManualTimeConstructorValid()
{
    mip::Nanoseconds base(500);
    mip::Seconds templated(500);
    
    try
    {
        mip::TimestampExperimental(mip::UnixTime(), base);
        mip::TimestampExperimental(mip::UnixTime(), templated);
    }
    catch(const std::invalid_argument&)
    {
        outputFailed("ManualConstructor", "invalid_argument raised when inputs are valid");
        return false;
    }

    return true; 
}

bool testGetTimestamp()
{
    std::array<mip::Nanoseconds, 3> test_values{
        mip::Nanoseconds(0), 
        mip::Nanoseconds(nanoseconds_in_week - 500), 
        mip::Nanoseconds(nanoseconds_in_week + 500)
    };
    
    for (auto &value : test_values)
    {
        mip::TimestampExperimental timestamp(mip::UnixTime(), value); 
       
        auto base_actual = timestamp.getTimestamp();
        if (!getterTestCase("GetTimestamp-base", timestamp.getTimestamp(), value))
        {
            return false;
        }

        auto template_actual = timestamp.getTimestamp<mip::Seconds>();
        mip::Seconds template_expected = std::chrono::duration_cast<mip::Seconds>(value);
        if (!getterTestCase("GetTimestamp-template", template_actual, template_expected))
        {
            return false;
        }
    }
    
    return true;
}

bool testSynchronizeUnix()
{
    mip::TimestampExperimental unix_timestamp(mip::UnixTime());
    unix_timestamp.synchronize();

    mip::Nanoseconds actual = unix_timestamp.getTimestamp();
    mip::Nanoseconds expected = std::chrono::system_clock::now();
    if (!getterTestCase("Synchronize-unix", actual, expected))
    {
        return false;
    }
    
    return true;
}

bool testSynchronizeStandards()
{
    std::array<mip::TimeStandard, 2> standards{
        mip::UnixTime(),
        mip::GpsTime()
    };
    

    for (const mip::TimeStandard& standard : standards)
    {
        mip::TimestampExperimental timestamp(standard);
        timestamp.synchronize();

        mip::Nanoseconds actual = timestamp.getTimestamp();
        mip::Nanoseconds expected = standard.now();
        getterTestCase("Synchronize-standards", actual, expected);
    }

    return true;
}

bool testNow()
{
    // TODO: Implement
    return true;
}
    //     mip::TimestampManager timestamp(standard);
    //     mip::Nanoseconds synced_time = timestamp.getTimestamp();

    //     if (synced_time == mip::Nanoseconds(0))
    //     {
    //         outputCaseResults("SyncConstructor-GPS", synced_time.count(), "time since epoch >= 0");
    //         return false;
    //     }
        
    //     static constexpr short leap = 18; 
    //     static constexpr int epoch_gap = 315964800; 
    // }


    //     if (!getterTestCase("GetTimeOfWeek-base", timestamp.getTimeOfWeek(), 
    //         mip::Nanoseconds(value % nanoseconds_in_week)))
    //     {
    //         return false;
    //     }

    //     if (!getterTestCase("GetTimeOfWeek-template", timestamp.getTimeOfWeek<mip::Seconds>(), 
    //         mip::Seconds(toSeconds(value) % seconds_in_week)))
    //     {
    //         return false;
    //     }
    // }
// }

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
