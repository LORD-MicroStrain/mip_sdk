#include <functional>
#include <iterator>
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
constexpr mip::Weeks       invalid_weeks_lower(-1);
constexpr mip::Weeks       invalid_weeks_upper(weeks_in_year + mip::Weeks(1));
constexpr mip::Nanoseconds main_test_nanoseconds(123456789);
constexpr mip::Seconds     main_test_seconds(123456789);
constexpr mip::Nanoseconds more_than_week(nanoseconds_in_week + nanoseconds_in_second);
constexpr mip::Nanoseconds half_week_nanoseconds(nanoseconds_in_week / 2);
constexpr mip::Seconds     half_week_seconds(seconds_in_week / 2);
constexpr mip::Seconds     quarter_week_seconds(seconds_in_week / 4);

/** Test case utilities *****************************************************************/

template<typename T1, typename T2>
bool getterTestCase(T1 actual, T2 expected);

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

/** Mocks *******************************************************************************/

struct MockUnixTime : mip::UnixTime
{
    mip::Nanoseconds now() const override
    {
        return main_test_nanoseconds;
    }
};
    
struct MockIncrement : mip::UnixTime
{
    MockIncrement()
    {
        for (int i = 0; i < 10; ++i)
        {
            values.push_back(mip::Nanoseconds(i));
        }

       current_value = values.begin(); 
    }

    mip::Nanoseconds now() const override
    {
        mip::Nanoseconds curr = *current_value;
        if (current_value != values.end())
        {
            std::advance(current_value, 1);
        }

        return curr;
    }

    std::vector<mip::Nanoseconds> values;
    std::vector<mip::Nanoseconds>::iterator current_value;
};

/** Setup *******************************************************************************/

mip::TimestampExperimental setupTimestampZero()
{
    return mip::TimestampExperimental(mip::UnixTime{}, mip::Nanoseconds(0)); 
}

mip::TimestampExperimental setupTimestampOneNanosecond()
{
    return mip::TimestampExperimental(mip::UnixTime{}, mip::Nanoseconds(1)); 
}

mip::TimestampExperimental setupTimestampOneSecond()
{
    return mip::TimestampExperimental(mip::UnixTime{}, mip::Seconds(1)); 
}

mip::TimestampExperimental setupTimestampOneSecondPlusNanosecond()
{
    return mip::TimestampExperimental(mip::UnixTime{}, nanoseconds_in_second + mip::Nanoseconds(1)); 
}

mip::TimestampExperimental setupTimestampOneWeek()
{
    return mip::TimestampExperimental(mip::UnixTime{}, mip::Weeks(1)); 
}

mip::TimestampExperimental setupTimestampHalfWeek()
{
    return mip::TimestampExperimental(mip::UnixTime{}, nanoseconds_in_week / 2); 
}

mip::TimestampExperimental setupTimestampMoreThanWeek()
{
    return mip::TimestampExperimental(mip::UnixTime{}, more_than_week); 
}

mip::TimestampExperimental setupTimestampMockUnixZero()
{
    return mip::TimestampExperimental(MockUnixTime{}, mip::Nanoseconds(0)); 
}

mip::TimestampExperimental setupTimestampMockUnixSynced()
{
    return mip::TimestampExperimental(MockUnixTime{}); 
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

    suite.addTest("SetTimestampInvalidBase", []() -> bool
    {
        auto timestamp = setupTimestampZero();

        return invalidInputTestCase<std::invalid_argument>([&timestamp]() -> void
        {
            timestamp.setTimestamp(invalid_nanoseconds);
        });
    });

    suite.addTest("SetTimestampInvalidTemplate", []() -> bool
    {
        auto timestamp = setupTimestampZero();

        return invalidInputTestCase<std::invalid_argument>([&timestamp]() -> void
        {
            timestamp.setTimestamp(invalid_seconds);
        });
    });

    suite.addTest("SetTimestampBase", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        timestamp.setTimestamp(main_test_nanoseconds);
        return getterTestCase(timestamp.getTimestamp(), main_test_nanoseconds);
    });

    suite.addTest("SetTimestampTemplate", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        timestamp.setTimestamp<mip::Seconds>(main_test_seconds);
        return getterTestCase(timestamp.getTimestamp<mip::Seconds>(), main_test_seconds);
    });
    
    suite.addTest("TestSynchronize", []() -> bool
    {
        auto timestamp = setupTimestampMockUnixZero();
        
        timestamp.synchronize();
        return getterTestCase(timestamp.getTimestamp(), main_test_nanoseconds);
    });

    suite.addTest("TestSynchronizeConstructor", []() -> bool
    {
        mip::TimestampExperimental timestamp(MockUnixTime{});
        return getterTestCase(timestamp.getTimestamp(), main_test_nanoseconds);
    });

    suite.addTest("SetWeekInvalidLower", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        return invalidInputTestCase<std::invalid_argument>([&timestamp]() -> void
        {
            timestamp.setWeek(invalid_weeks_lower);
        });
    });

    suite.addTest("SetWeekLower", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        timestamp.setWeek(mip::Weeks(0));
        return getterTestCase(timestamp.getTimestamp<mip::Weeks>(), mip::Weeks(0));
    });

    suite.addTest("SetWeekUpper", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        timestamp.setWeek(weeks_in_year);
        return getterTestCase(timestamp.getTimestamp<mip::Weeks>(), weeks_in_year);
    });

    suite.addTest("SetWeekMain", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        timestamp.setWeek(mip::Weeks(30));
        return getterTestCase(timestamp.getTimestamp<mip::Weeks>(), mip::Weeks(30));
    });

    suite.addTest("GetTimeOfWeekInvalidTemplateDuration", []() -> bool
    {
        auto timestamp = setupTimestampMoreThanWeek();
        
        return invalidInputTestCase<std::invalid_argument>([&timestamp]() -> void 
        {
            timestamp.getTimeOfWeek<mip::Weeks>();
        });
    });

    suite.addTest("GetTimeOfWeekEqualBase", []() -> bool
    {
        auto timestamp = setupTimestampOneWeek();
        
        return getterTestCase(timestamp.getTimeOfWeek(), mip::Nanoseconds(0));
    });

    suite.addTest("GetTimeOfWeekEqualTemplate", []() -> bool
    {
        auto timestamp = setupTimestampOneWeek();
        
        return getterTestCase(timestamp.getTimeOfWeek<mip::Seconds>(), mip::Seconds(0));
    });

    suite.addTest("GetTimeOfWeekLessBase", []() -> bool
    {
        auto timestamp = setupTimestampHalfWeek();
        
        return getterTestCase(timestamp.getTimeOfWeek(), half_week_nanoseconds);
    });

    suite.addTest("GetTimeOfWeekLessTemplate", []() -> bool
    {
        auto timestamp = setupTimestampHalfWeek();
        
        return getterTestCase(timestamp.getTimeOfWeek<mip::Seconds>(), half_week_seconds);
    });

    suite.addTest("GetTimeOfWeekMoreBase", []() -> bool
    {
        auto timestamp = setupTimestampMoreThanWeek();
        
        return getterTestCase(timestamp.getTimeOfWeek(), nanoseconds_in_second);
    });

    suite.addTest("GetTimeOfWeekMoreTemplate", []() -> bool
    {
        auto timestamp = setupTimestampMoreThanWeek();
        
        return getterTestCase(timestamp.getTimeOfWeek<mip::Seconds>(), mip::Seconds(1));
    });

    suite.addTest("SetTimeOfWeekInvalidLowerBase", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        return invalidInputTestCase<std::invalid_argument>([&timestamp]() -> void 
        {
            timestamp.setTimeOfWeek(invalid_nanoseconds);
        });
    });

    suite.addTest("SetTimeOfWeekInvalidLowerTemplate", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        return invalidInputTestCase<std::invalid_argument>([&timestamp]() -> void 
        {
            timestamp.setTimeOfWeek(invalid_seconds);
        });
    });

    suite.addTest("SetTimeOfWeekInvalidUpperBase", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        return invalidInputTestCase<std::invalid_argument>([&timestamp]() -> void 
        {
            timestamp.setTimeOfWeek(nanoseconds_in_week);
        });
    });

    suite.addTest("SetTimeOfWeekInvalidUpperTemplate", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        return invalidInputTestCase<std::invalid_argument>([&timestamp]() -> void 
        {
            timestamp.setTimeOfWeek(mip::Weeks(1));
        });
    });

    suite.addTest("SetTimeOfWeekLower", []() -> bool
    {
        auto timestamp = setupTimestampOneSecond();
        
        timestamp.setTimeOfWeek(mip::Nanoseconds(0));
        return getterTestCase(timestamp.getTimeOfWeek(), mip::Nanoseconds(0));
    });

    suite.addTest("SetTimeOfWeekUpper", []() -> bool
    {
        constexpr mip::Nanoseconds timeOfWeekUpperBound(nanoseconds_in_week - mip::Nanoseconds(1));
        auto timestamp = setupTimestampZero();
        
        timestamp.setTimeOfWeek(timeOfWeekUpperBound);
        return getterTestCase(timestamp.getTimeOfWeek(), timeOfWeekUpperBound);
    });

    suite.addTest("SetTimeOfWeekArbitraryBase", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        timestamp.setTimeOfWeek(half_week_nanoseconds);
        return getterTestCase(timestamp.getTimeOfWeek(), half_week_nanoseconds);
    });

    suite.addTest("SetTimeOfWeekArbitraryTemplate", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        timestamp.setTimeOfWeek(quarter_week_seconds);
        return getterTestCase(timestamp.getTimeOfWeek<mip::Seconds>(), quarter_week_seconds);
    });

    suite.addTest("TimeElapsedInvalidBase", []() -> bool
    {
        auto higher = setupTimestampZero();
        auto lower = setupTimestampOneNanosecond(); // Intentionally higher!
        
        return invalidInputTestCase<std::invalid_argument>([&higher, &lower]() -> void 
        {
            higher.timeElapsed(lower);
        });
    });

    suite.addTest("TimeElapsedInvalidTemplate", []() -> bool
    {
        auto higher = setupTimestampZero();
        auto lower = setupTimestampOneNanosecond(); // Intentionally higher and in nanoseconds!
        
        return invalidInputTestCase<std::invalid_argument>([&higher, &lower]() -> void 
        {
            higher.timeElapsed<mip::Seconds>(lower);
        });
    });

    suite.addTest("TimeElapsedSameBase", []() -> bool
    {
        auto timestamp1 = setupTimestampOneNanosecond();
        auto timestamp2 = setupTimestampOneNanosecond();
        
        return getterTestCase(timestamp1.timeElapsed(timestamp2), false);
    });

    suite.addTest("TimeElapsedSameTemplate", []() -> bool
    {
        auto timestamp1 = setupTimestampOneSecond();
        auto timestamp2 = setupTimestampOneSecond();
        
        return getterTestCase(timestamp1.timeElapsed<mip::Seconds>(timestamp2), false);
    });

    suite.addTest("TimeElapsedOneBase", []() -> bool
    {
        mip::TimestampExperimental higher(mip::UnixTime{}, mip::Nanoseconds(2));
        auto lower = setupTimestampOneNanosecond();
        
        return getterTestCase(higher.timeElapsed(lower), true);
    });

    suite.addTest("TimeElapsedOneTemplate", []() -> bool
    {
        mip::TimestampExperimental higher(mip::UnixTime{}, mip::Seconds(2));
        auto lower = setupTimestampOneSecond();
        
        return getterTestCase(higher.timeElapsed<mip::Seconds>(lower), true);
    });

    suite.addTest("TimeElapsedArbitrary", []() -> bool
    {
        mip::TimestampExperimental higher(mip::UnixTime{}, nanoseconds_in_second + mip::Nanoseconds(1));
        auto lower = setupTimestampOneSecond();
        
        bool success = true;
        success &= getterTestCase(higher.timeElapsed(lower), true);
        success &= getterTestCase(higher.timeElapsed<mip::Seconds>(lower), false);
        return success;
    });

    suite.addTest("TimeChangedInvalidBase", []() -> bool
    {
        auto higher = setupTimestampZero();
        auto lower = setupTimestampOneNanosecond(); // Intentionally higher!
        
        return invalidInputTestCase<std::invalid_argument>([&higher, &lower]() -> void 
        {
            higher.timeChanged(lower);
        });
    });

    suite.addTest("TimeChangedInvalidTemplate", []() -> bool
    {
        auto higher = setupTimestampZero();
        auto lower = setupTimestampOneNanosecond(); // Intentionally higher and in nanoseconds!
        
        return invalidInputTestCase<std::invalid_argument>([&higher, &lower]() -> void 
        {
            higher.timeChanged<mip::Seconds>(lower);
        });
    });

    suite.addTest("TimeChangedSameBase", []() -> bool
    {
        auto timestamp1 = setupTimestampOneNanosecond();
        auto timestamp2 = setupTimestampOneNanosecond();
        
        return getterTestCase(timestamp1.timeChanged(timestamp2), false);
    });

    suite.addTest("TimeChangedSameTemplate", []() -> bool
    {
        auto timestamp1 = setupTimestampOneSecond();
        auto timestamp2 = setupTimestampOneSecond();
        
        return getterTestCase(timestamp1.timeChanged<mip::Seconds>(timestamp2), false);
    });

    suite.addTest("TimeChangedOneBase", []() -> bool
    {
        mip::TimestampExperimental higher(mip::UnixTime{}, mip::Nanoseconds(2));
        auto lower = setupTimestampOneNanosecond();
        
        return getterTestCase(higher.timeChanged(lower), true);
    });

    suite.addTest("TimeChangedOneTemplate", []() -> bool
    {
        mip::TimestampExperimental higher(mip::UnixTime{}, mip::Seconds(2));
        auto lower = setupTimestampOneSecond();
        
        return getterTestCase(higher.timeChanged<mip::Seconds>(lower), true);
    });

    suite.addTest("TimeChangedArbitrary", []() -> bool
    {
        auto higher = setupTimestampOneSecondPlusNanosecond();
        auto lower = setupTimestampOneSecond();
        
        bool success = true;
        success &= getterTestCase(higher.timeChanged(lower), true);
        success &= getterTestCase(higher.timeChanged<mip::Seconds>(lower), false);
        return success;
    });

    suite.addTest("TimeElapsedChangedDifferentiation", []() -> bool
    {
        auto higher = setupTimestampOneSecondPlusNanosecond();
        mip::TimestampExperimental lower(mip::UnixTime{}, nanoseconds_in_second - mip::Nanoseconds(1));
        
        // Should be false for timeElapsed() (since a full second doesn't elapse).
        // Should be true for timeChanged() though, since they are in different second intervals.
        bool success = true;
        success &= getterTestCase(higher.timeElapsed<mip::Seconds>(lower), false);
        success &= getterTestCase(higher.timeChanged<mip::Seconds>(lower), true);
        return success;
    });
    
    suite.addTest("CastTimeInvalidLower", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        
        return invalidInputTestCase<std::invalid_argument>([&timestamp]() -> void 
        {
            timestamp.castTime<std::int32_t>(invalid_nanoseconds);
        });
    });

    suite.addTest("CastTimeZero", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        std::int32_t zero_count = 0;
        
        return getterTestCase(timestamp.castTime<std::int32_t>(timestamp.getTimestamp()), zero_count);
    });

    suite.addTest("CastTimeArbitrary", []() -> bool
    {
        auto timestamp = setupTimestampOneWeek();
        std::int32_t seconds_count = 604800;
        
        return getterTestCase(timestamp.castTime<std::int32_t>(timestamp.getTimestamp<mip::Seconds>()), seconds_count);
    });

    suite.addTest("IncrementInvalid", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        auto reference_synced = setupTimestampHalfWeek();
        auto reference_old = setupTimestampMoreThanWeek(); // Intentionally higher!
        
        return invalidInputTestCase<std::invalid_argument>([&]() -> void 
        {
            timestamp.increment(reference_synced, reference_old);
        });
    });

    suite.addTest("IncrementNoChange", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        auto reference_synced = setupTimestampMockUnixSynced();
        auto reference_old = setupTimestampMockUnixSynced();

        timestamp.increment(reference_synced, reference_old);
        return getterTestCase(timestamp.getTimestamp(), mip::Nanoseconds(0));
        // // First increment should change to the main test.
        // // Second increment shouldn't change, as the mock time standard only returns one value.
        // bool success = true;
        // for (int i = 0; i < 2; ++i)
        // {
        //     timestamp.syncToReference(reference);
        //     success &= getterTestCase(timestamp.getTimestamp(), main_test_nanoseconds);
        // }
        
        // return success;
    });

    suite.addTest("IncrementChange", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        auto reference_old = setupTimestampZero();

        MockIncrement mock{};
        mip::TimestampExperimental reference_synced(mock, mip::Nanoseconds(0));
        
        for (int i = 0; i < mock.values.size(); ++i)
        {
            timestamp.increment(reference_synced, reference_old);
            if (!getterTestCase(timestamp.getTimestamp(), mock.values[i]))
            {
                return false;
            }
            
            reference_old.setTimestamp(reference_synced.getTimestamp());
            reference_synced.synchronize();
        }
        
        return true;
    });

    return suite.run();
}


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

template<typename T1, typename T2>
bool getterTestCase(T1 actual, T2 expected)
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

template<typename Rep, typename Period>
std::string format(std::chrono::duration<Rep, Period> input)
{
    return std::to_string(input.count());
}

template<typename T>
std::string format(T input)
{
    return std::to_string(input);
}

std::string format(bool input)
{
    return input ? "True" : "False";
}

// Required to support format call.
// TODO: Find better solution that avoids needing this?
const char * format(const char *input)
{
    return input;
}

template<typename T1, typename T2>
void outputCaseResults(T1 actual, T2 expected)
{
    std::cout << 
        "Failed!\n" <<
        "---> Actual:   " << format(actual) << "\n" <<
        "---> Expected: " << format(expected) << "\n";
}

void outputFailed(const char* message)
{
    std::cerr << 
        "Failed!\n" << 
        "---> " << message << "\n";
}
