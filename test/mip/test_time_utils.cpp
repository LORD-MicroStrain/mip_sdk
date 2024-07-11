#include <functional>
#include <iostream>
#include <unordered_map>

#include <mip/utils/timestamp.hpp>

// Test evalutation
constexpr std::uint8_t success = 0;
constexpr std::uint8_t fail = 1;

// Time conversions
constexpr mip::Nanoseconds nanoseconds_in_second(1000000000);
constexpr mip::Seconds     seconds_in_week(604800);
constexpr mip::Nanoseconds nanoseconds_in_week(nanoseconds_in_second * seconds_in_week.count());
constexpr mip::Weeks       weeks_in_year(52);

// Test values
constexpr mip::Nanoseconds invalid_nanoseconds(-1);
constexpr mip::Seconds     invalid_seconds(-1);
constexpr mip::Weeks       invalid_weeks(-1);
constexpr mip::Nanoseconds main_test_nanoseconds(123456789);
constexpr mip::Seconds     main_test_seconds(123456789);
constexpr mip::Nanoseconds more_than_week(nanoseconds_in_week + nanoseconds_in_second);
constexpr mip::Nanoseconds half_week_nanoseconds(nanoseconds_in_week / 2);
constexpr mip::Seconds     quarter_week_seconds(seconds_in_week / 4);

/** Mock objects ************************************************************************/

struct MockUnixTime : mip::UnixTime
{
    mip::Nanoseconds now() const override
    {
        return main_test_nanoseconds;
    }
};

/** Test case utilities *****************************************************************/

template<typename Duration1, typename Duration2>
bool getterTestCase(Duration1 actual, Duration2 expected);
bool getterTestCase(bool actual, bool expected);

template<typename ExpectedException, typename Callable>
bool invalidInputTestCase(Callable test_wrapper);

// Outputs name of running test.
void outputRunning(const char *name);

// Comprehensive failed message with actual vs. expected values.
template<typename T1, typename T2>
void outputCaseResults(T1 actual, T2 expected);

// Simple failed message
void outputFailed(const char* message);

// TODO: Move to test utilities file.
class TestSuite
{
public:
    TestSuite() {}

    /// Test should return true if succeeded, or false if failed.
    void addTest(const char *name, std::function<bool()> test);

    /// Call at the end of the 'main' function.
    int run();

private:
    std::vector<std::tuple<const char *, std::function<bool()>>> tests; 
};

/** Setup *******************************************************************************/

mip::TimestampExperimental setupTimestampZero()
{
    return mip::TimestampExperimental(mip::UnixTime{}, mip::Nanoseconds(0)); 
}

mip::TimestampExperimental setupTimestampOneSecond()
{
    return mip::TimestampExperimental(mip::UnixTime{}, mip::Seconds(1)); 
}

/** Tests *******************************************************************************/

int main(int argc, const char* argv[])
{
    TestSuite suite{};

    suite.addTest("ManualConstructorInvalidBase", []() -> bool
    {
        return invalidInputTestCase<std::invalid_argument>([]() -> void 
        {
            mip::TimestampExperimental(mip::UnixTime{}, invalid_nanoseconds);
        });
    });

    suite.addTest("ManualConstructorInvalidTemplate", []() -> bool
    {
        return invalidInputTestCase<std::invalid_argument>([]() -> void 
        {
            mip::TimestampExperimental(mip::UnixTime{}, invalid_seconds);
        });
    });

    suite.addTest("GetTimestampZero", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        return getterTestCase(timestamp.getTimestamp(), mip::Nanoseconds(0));
    });

    suite.addTest("GetTimestampBase", []() -> bool
    {
        auto timestamp = setupTimestampOneSecond();
        return getterTestCase(timestamp.getTimestamp(), nanoseconds_in_second);
    });

    suite.addTest("GetTimestampTemplate", []() -> bool
    {
        auto timestamp = setupTimestampOneSecond();
        return getterTestCase(timestamp.getTimestamp<mip::Seconds>(), mip::Seconds(1));
    });

    return suite.run();
}

bool testSetTimestampInvalidBase()
{
//     mip::TimestampExperimental timestamp(mip::UnixTime{});

//     auto invalid_base = [&timestamp]() -> void { timestamp.setTimestamp(invalid_nanoseconds); };
//     if (!invalidInputTestCase<std::invalid_argument>("SetTimestampInvalid-base", invalid_base))
//     {
//         return false;
//     }
    outputRunning(__FUNCTION__);
    
    return false;
}

bool testSetTimestampInvalidTemplate()
{
//     auto invalid_templated = [&timestamp]() -> void { timestamp.setTimestamp(invalid_seconds); };
//     if (!invalidInputTestCase<std::invalid_argument>("SetTimestampInvalid-template", invalid_templated))
//     {
//         return false;
//     }

    outputRunning(__FUNCTION__);
    
    return false;
}

bool testSetTimestampBase()
{
//     timestamp.setTimestamp(main_test_nanoseconds);
//     if (!getterTestCase("SetTimestamp-base", timestamp.getTimestamp(), main_test_nanoseconds))
//     {
//         return false;
//     }
    
    outputRunning(__FUNCTION__);
    
    return false;
}

bool testSetTimestampTemplate()
{
//     timestamp.setTimestamp(main_test_seconds);
//     if (!getterTestCase("SetTimestamp-template", timestamp.getTimestamp<mip::Seconds>(), main_test_seconds))
//     {
//         return false;
//     }

//     return true;
// }

    return false; 
}

// bool testSynchronize()
// {
//     mip::TimestampExperimental timestamp(MockUnixTime{});

//     timestamp.synchronize();
//     if (!getterTestCase("Synchronize", timestamp.getTimestamp(), main_test_nanoseconds))
//     {
//         return false;
//     }
    
//     return true;
// }

// bool testNow()
// {
//     mip::TimestampExperimental timestamp = mip::TimestampExperimental::Now(MockUnixTime());
    
//     if (!getterTestCase("Now", timestamp.getTimestamp(), main_test_nanoseconds))
//     {
//         return false;
//     }

//     return true;
// }

// bool testSetWeek()
// {
//     mip::TimestampExperimental timestamp(mip::UnixTime{}, mip::Nanoseconds(0));

//     auto invalid_lower = [&timestamp]() -> void { timestamp.setWeek(invalid_weeks); };
//     if (!invalidInputTestCase<std::invalid_argument>("SetWeek-invalid-lower", invalid_lower))
//     {
//         return false;
//     }

//     auto invalid_upper = [&timestamp]() -> void { timestamp.setWeek(weeks_in_year + mip::Weeks(1)); };
//     if (!invalidInputTestCase<std::invalid_argument>("SetWeek-invalid-upper", invalid_upper))
//     {
//         return false;
//     }
    
//     timestamp.setWeek(mip::Weeks(0));
//     if (!getterTestCase("SetWeek-zero", timestamp.getTimestamp<mip::Weeks>(), mip::Weeks(0)))
//     {
//         return false;
//     }

//     timestamp.setWeek(weeks_in_year);
//     if (!getterTestCase("SetWeek-upper", timestamp.getTimestamp<mip::Weeks>(), weeks_in_year))
//     {
//         return false;
//     }

//     timestamp.setWeek(mip::Weeks(30));
//     if (!getterTestCase("SetWeek-main", timestamp.getTimestamp<mip::Weeks>(), mip::Weeks(30)))
//     {
//         return false;
//     }
    
//     return true;
// }

// bool testGetTimeOfWeek()
// {
//     mip::TimestampExperimental timestamp(mip::UnixTime{}, nanoseconds_in_week);
    
//     if (!getterTestCase("GetTimeOfWeek-equal-base", timestamp.getTimeOfWeek(), mip::Nanoseconds(0)))
//     {
//         return false;
//     }

//     if (!getterTestCase("GetTimeOfWeek-equal-template", timestamp.getTimeOfWeek<mip::Seconds>(), mip::Seconds(0)))
//     {
//         return false;
//     }

//     timestamp.setTimestamp(nanoseconds_in_second);

//     if (!getterTestCase("GetTimeOfWeek-less-base", timestamp.getTimeOfWeek(), nanoseconds_in_second))
//     {
//         return false;
//     }

//     if (!getterTestCase("GetTimeOfWeek-less-template", timestamp.getTimeOfWeek<mip::Seconds>(), mip::Seconds(1)))
//     {
//         return false;
//     }

//     timestamp.setTimestamp(more_than_week);

//     if (!getterTestCase("GetTimeOfWeek-more-base", timestamp.getTimeOfWeek(), nanoseconds_in_second))
//     {
//         return false;
//     }

//     if (!getterTestCase("GetTimeOfWeek-more-template", timestamp.getTimeOfWeek<mip::Seconds>(), mip::Seconds(1)))
//     {
//         return false;
//     }

//     return true;
// }

// bool testSetTimeOfWeek()
// {
//     mip::TimestampExperimental timestamp(mip::UnixTime{}); 
    
//     auto invalid_lower_base = [&timestamp]() -> void { timestamp.setTimeOfWeek(invalid_nanoseconds); };
//     if (!invalidInputTestCase<std::invalid_argument>("SetTimeOfWeek-invalid-lower-base", invalid_lower_base))
//     {
//         return false;
//     }

//     auto invalid_lower_template = [&timestamp]() -> void { timestamp.setTimeOfWeek(invalid_seconds); };
//     if (!invalidInputTestCase<std::invalid_argument>("SetTimeOfWeek-invalid-lower-template", invalid_lower_template))
//     {
//         return false;
//     }

//     auto invalid_upper_base = [&timestamp]() -> void { timestamp.setTimeOfWeek(nanoseconds_in_week + mip::Nanoseconds(1)); };
//     if (!invalidInputTestCase<std::invalid_argument>("SetTimeOfWeek-invalid-upper-base", invalid_upper_base))
//     {
//         return false;
//     }

//     auto invalid_upper_template = [&timestamp]() -> void { timestamp.setTimeOfWeek(seconds_in_week + mip::Seconds(1)); };
//     if (!invalidInputTestCase<std::invalid_argument>("SetTimeOfWeek-invalid-upper-template", invalid_upper_template))
//     {
//         return false;
//     }

//     timestamp.setTimeOfWeek(half_week_nanoseconds);
//     if (!getterTestCase("SetTimeOfWeek-base", timestamp.getTimeOfWeek(), half_week_nanoseconds))
//     {
//         return false;
//     }

//     timestamp.setTimeOfWeek(quarter_week_seconds);
//     if (!getterTestCase("SetTimeOfWeek-template", timestamp.getTimeOfWeek<mip::Seconds>(), quarter_week_seconds))
//     {
//         return false;
//     }

//     return true; 
// }

// bool testTimeElapsed()
// {
//     std::string current_test{""};
// try {
//     mip::TimestampExperimental higher(mip::UnixTime{}, mip::Nanoseconds(0));
//     mip::TimestampExperimental lower(mip::UnixTime{}, mip::Nanoseconds(1));  // Intentionally higher!

//     current_test = "TimeElapsed-invalid-base";
//     auto invalid_base = [&higher, &lower]() -> void { higher.timeElapsed(lower); };
//     if (!invalidInputTestCase<std::invalid_argument>(current_test.c_str(), invalid_base))
//     {
//         return false;
//     }

//     current_test = "TimeElapsed-invalid-template";
//     auto invalid_template = [&higher, &lower]() -> void { higher.timeElapsed<mip::Seconds>(lower); };
//     if (!invalidInputTestCase<std::invalid_argument>(current_test.c_str(), invalid_template))
//     {
//         return false;
//     }
    
//     current_test = "TimeElapsed-same-base";
//     higher.setTimestamp(mip::Nanoseconds(1));
//     if (!getterTestCase(current_test.c_str(), higher.timeElapsed(lower), false))
//     {
//         return false;
//     }

//     current_test = "TimeElapsed-same-template";
//     higher.setTimestamp(mip::Seconds(1));
//     lower.setTimestamp(mip::Seconds(1));
//     if (!getterTestCase(current_test.c_str(), higher.timeElapsed<mip::Seconds>(lower), false))
//     {
//         return false;
//     }

//     current_test = "TimeElapsed-one-base";
//     higher.setTimestamp(mip::Nanoseconds(2));
//     lower.setTimestamp(mip::Nanoseconds(1));
//     if (!getterTestCase(current_test.c_str(), higher.timeElapsed(lower), true))
//     {
//         return false;
//     }

//     current_test = "TimeElapsed-one-template";
//     higher.setTimestamp(mip::Seconds(2));
//     lower.setTimestamp(mip::Seconds(1));
//     if (!getterTestCase(current_test.c_str(), higher.timeElapsed<mip::Seconds>(lower), true))
//     {
//         return false;
//     }

//     current_test = "TimeElapsed-main-base";
//     higher.setTimestamp(nanoseconds_in_second + mip::Nanoseconds(1));
//     lower.setTimestamp(nanoseconds_in_second);
//     if (!getterTestCase(current_test.c_str(), higher.timeElapsed(lower), true))
//     {
//         return false;
//     }

//     current_test = "TimeElapsed-main-base";
//     if (!getterTestCase(current_test.c_str(), higher.timeElapsed<mip::Seconds>(lower), false))
//     {
//         return false;
//     }
// }
// catch (std::exception &e)
// {
//     std::cerr << "Unexpected exception occurred: " << "\n";
//     std::cerr << current_test.c_str() << " ---> " << e.what() << "\n";    
//     return false;
// }

//     return true; 
// }


/**************************************************************************************/
/* NOTE: The following are definitions for all declarations above. There are no new   */
/*       declarations following this statement.                                       */
/**************************************************************************************/

void TestSuite::addTest(const char *name, std::function<bool()> test)
{
    tests.push_back(std::make_tuple(name, test));
}

int TestSuite::run()
{
    try
    {
        for (auto const &test : tests)
        {
            const char *name = std::get<0>(test);
            std::function<bool()> test_callable = std::get<1>(test);

            outputRunning(name);
            if (!test_callable())
            {
                return fail;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "Unexpected exception occurred: \n";
        std::cerr << e.what() << '\n';
        return fail;
    }

    return success;
}   

template<typename Duration1, typename Duration2>
bool getterTestCase(Duration1 actual, Duration2 expected)
{
    const std::type_info& actual_type = typeid(actual);
    const std::type_info& expected_type = typeid(expected);
    
    if (actual_type != expected_type)
    {
        std::cout << "Type ";
        outputCaseResults(actual_type.name(), expected_type.name());
        return false;
    }

    if (actual != expected)
    {
        std::cout << "Value ";
        outputCaseResults(actual.count(), expected.count());
        return false;
    }
    
    return true;
}

bool getterTestCase(bool actual, bool expected)
{
    if (actual != expected)
    {
        outputCaseResults(actual, expected);
        return false;
    }
    
    return true;
}

template<typename ExpectedException, typename Callable>
bool invalidInputTestCase(Callable test_wrapper)
{
    try
    {
        test_wrapper();
    }
    catch(const ExpectedException&)
    {
        return true;
    }
    
    outputFailed("No exception raised for invalid input.");
    return false;
}

void outputRunning(const char *name)
{
    std::cout << "Running: " << name << "\n";
}

template<typename T1, typename T2>
void outputCaseResults(T1 actual, T2 expected)
{
    std::cout << 
        "Failed!\n" <<
        "---> Actual:   " << actual << "\n" <<
        "---> Expected: " << expected << "\n";
}

void outputFailed(const char* message)
{
    std::cerr << 
        "Failed!\n" << 
        "---> " << message << "\n";
}
