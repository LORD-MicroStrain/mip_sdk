#include <functional>
#include <iterator>
#include <iostream>
#include <unordered_map>
#include <string>

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
void outputTestName(const char *name);
// Outputs name of running sub-test (single test case).
void outputSubtestName(const char *name);

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

struct MockTimeConvertOneSecond : mip::TimeStandard
{
    mip::Nanoseconds now() const override
    {
        return mip::Nanoseconds(0); // Placeholder
    }

    mip::Nanoseconds convertToBase(mip::Nanoseconds time) const override
    {
        return time - mip::Seconds(1);
    }

    mip::Nanoseconds convertFromBase(mip::Nanoseconds time) const override
    {
        return time + mip::Seconds(1);
    }
};

/** Setup *******************************************************************************/

// Time standards
const mip::UnixTime unix{};
const MockUnixTime mock_unix{};
const MockTimeConvertOneSecond mock_time_convert_one_second{};

mip::TimestampExperimental setupTimestampZero()
{
    return {unix, mip::Nanoseconds(0)};
}

mip::TimestampExperimental setupTimestampOneNanosecond()
{
    return {unix, mip::Nanoseconds(1)};
}

mip::TimestampExperimental setupTimestampOneSecond()
{
    return {unix, mip::Seconds(1)};
}

mip::TimestampExperimental setupTimestampOneSecondPlusNanosecond()
{
    return {unix, nanoseconds_in_second + mip::Nanoseconds(1)};
}

mip::TimestampExperimental setupTimestampOneWeek()
{
    return {unix, mip::Weeks(1)};
}

mip::TimestampExperimental setupTimestampHalfWeek()
{
    return {unix, nanoseconds_in_week / 2};
}

mip::TimestampExperimental setupTimestampMoreThanWeek()
{
    return {unix, more_than_week}; 
}

mip::TimestampExperimental setupTimestampMockUnixZero()
{
    return {mock_unix, mip::Nanoseconds(0)}; 
}

mip::TimestampExperimental setupTimestampMockUnixSynced()
{
    return {mock_unix};
}

mip::TimestampExperimental setupTimestampMockConvertOneSecond(mip::Nanoseconds time = mip::Nanoseconds(0))
{
    return {mock_time_convert_one_second, time}; 
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

    suite.addTest("GetTimestampBaseStandardZero", []() -> bool
    {
        auto timestamp = setupTimestampMockConvertOneSecond(mip::Seconds(1));

        return getterTestCase(timestamp.getTimestampBaseStandard(), mip::Nanoseconds(0));
    });

    // TODO: Rename tests so this isn't jank lol.
    suite.addTest("GetTimestampBaseStandardBase", []() -> bool
    {
        auto timestamp = setupTimestampMockConvertOneSecond(half_week_nanoseconds);

        return getterTestCase(timestamp.getTimestampBaseStandard(), half_week_nanoseconds - mip::Seconds(1));
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

    suite.addTest("SetTimestampConvertLowerZero", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        // This will cause the time to become negative after conversion to base.
        auto reference = setupTimestampMockConvertOneSecond(mip::Nanoseconds(500)); // Arbitrary value

        return invalidInputTestCase<std::logic_error>([&]() -> void 
        {
            timestamp.setTimestamp(reference);
        });
    });

    suite.addTest("SetTimestampConvert", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        auto reference = setupTimestampMockConvertOneSecond(nanoseconds_in_week);

        timestamp.setTimestamp(reference);
        return getterTestCase(timestamp.getTimestamp(), nanoseconds_in_week - mip::Seconds(1));
    });

    suite.addTest("SetTimestampConvertOtherWay", []() -> bool
    {
        auto timestamp = setupTimestampMockConvertOneSecond(mip::Nanoseconds(0));
        auto reference = setupTimestampOneWeek();

        timestamp.setTimestamp(reference);
        return getterTestCase(timestamp.getTimestamp(), nanoseconds_in_week + mip::Seconds(1));
    });
    
    suite.addTest("TestSynchronize", []() -> bool
    {
        auto timestamp = setupTimestampMockUnixZero();
        
        timestamp.synchronize();
        return getterTestCase(timestamp.getTimestamp(), main_test_nanoseconds);
    });

    suite.addTest("TestSynchronizeConstructor", []() -> bool
    {
        auto timestamp = setupTimestampMockUnixSynced();

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
        auto reference_synced = setupTimestampOneSecond();
        auto reference_old = setupTimestampOneSecond();

        timestamp.increment(reference_synced, reference_old);
        return getterTestCase(timestamp.getTimestamp(), mip::Nanoseconds(0));
    });

    suite.addTest("IncrementChange", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        auto reference_old = setupTimestampZero();
        auto reference_synced = setupTimestampOneNanosecond();
        
        outputSubtestName("OneNanosecond");
        timestamp.increment(reference_synced, reference_old);
        if (!getterTestCase(timestamp.getTimestamp(), mip::Nanoseconds(1)))
        {
            return false;
        }

        outputSubtestName("ThreeNanoseconds");
        timestamp.setTimestamp(mip::Nanoseconds(0));
        reference_synced.setTimestamp(mip::Nanoseconds(3));
        timestamp.increment(reference_synced, reference_old);
        if (!getterTestCase(timestamp.getTimestamp(), mip::Nanoseconds(3)))
        {
            return false;
        }

        outputSubtestName("FiveSeconds");
        timestamp.setTimestamp(mip::Nanoseconds(0));
        reference_old.setTimestamp(mip::Seconds(1));
        reference_synced.setTimestamp(mip::Seconds(6));
        timestamp.increment(reference_synced, reference_old);
        if (!getterTestCase(timestamp.getTimestamp<mip::Seconds>(), mip::Seconds(5)))
        {
            return false;
        }
        
        return true;
    });

    suite.addTest("ConvertSame", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        // Requires no conversion to base. Should return mocked now() value.
        auto reference = setupTimestampMockUnixSynced();
        
        return getterTestCase(timestamp.convertFrom(reference), main_test_nanoseconds);
    });

    suite.addTest("ConvertLowerZero", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        // This will cause the time to become negative after conversion to base.
        auto reference = setupTimestampMockConvertOneSecond(mip::Nanoseconds(500)); // Arbitrary value

        return invalidInputTestCase<std::logic_error>([&]() -> void 
        {
            timestamp.convertFrom(reference), mip::Nanoseconds(0);
        });
    });

    suite.addTest("ConvertBase", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        auto reference = setupTimestampMockConvertOneSecond(nanoseconds_in_week);

        return getterTestCase(timestamp.convertFrom(reference), nanoseconds_in_week - mip::Seconds(1));
    });

    suite.addTest("ConvertTemplate", []() -> bool
    {
        auto timestamp = setupTimestampZero();
        auto reference = setupTimestampMockConvertOneSecond(nanoseconds_in_week);

        return getterTestCase(timestamp.convertFrom<mip::Seconds>(reference), seconds_in_week - mip::Seconds(1));
    });

    suite.addTest("ConvertOtherWay", []() -> bool
    {
        auto timestamp = setupTimestampMockConvertOneSecond(mip::Nanoseconds(0));
        auto reference = setupTimestampOneWeek();

        return getterTestCase(timestamp.convertFrom(reference), nanoseconds_in_week + mip::Seconds(1));
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

            outputTestName(name);
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

void outputTestName(const char *name)
{
    std::cout << "Running test: " << name << "\n";
}

void outputSubtestName(const char *name)
{
    std::cout << "Running subtest: " << name << "\n";
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
