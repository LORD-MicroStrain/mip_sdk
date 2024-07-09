#pragma once

#include <assert.h>
#include <chrono>

#include "mip/utils/time_durations.hpp"
#include "mip/utils/time_standard.hpp"

namespace mip
{
    // TODO: Move to Timestamp.
    // TODO: Add duration changing.
    // template<typename DurationIn, typename DurationOut> 
    //     DurationOut convert(std::uint64_t time, TimeStandard to, 
    //     TimeStandard from); 
    // Nanoseconds setTimestamp(std::uint64_t time, TimeStandard to, 
    //     TimeStandard from)
    // Nanoseconds setTimestamp(std::uint64_t time, TimeStandard from)
        

    // TODO: Change Now to single standard constructor.
    // TODO: Add increment method.
    // TODO: Update documentation.
    // TODO: Change name to Timestamp when old one is removed throughout mip sdk.
    /// Manages a timestamp in nanoseconds since epoch.
    ///
    /// Notes:
    /// ----------------------------------------------------------------------------------
    ///     * Currently supports GPS clock (only) for the underlying time system.
    ///     * Might add support for different time systems in the future.
    ///     * Currently supports std::chrono:duration (only) for timestamp inputs.
    /// ----------------------------------------------------------------------------------
    class TimestampExperimental
    {
    public:
        TimestampExperimental() = delete;
        
        /// Defaults time since epoch to zero nanoseconds.
        TimestampExperimental(const TimeStandard &standard);

        /// Manually set time since epoch.
        template<typename DurationIn> 
        TimestampExperimental(const TimeStandard &standard, DurationIn time);

        /// Returns synchronized timestamp.
        static TimestampExperimental Now(const TimeStandard &standard);
        
        /// Synchronizes timestamp to a coordinated time standard. Continuously call this 
        /// method to keep the timestamp up to date with the time standard.
        void synchronize();

        /// Returns raw time since epoch.
        template<typename DurationOut> 
        DurationOut getTimestamp();
        Nanoseconds getTimestamp();
        
        // Sets raw time since epoch.
        template<typename DurationIn>
        void setTimestamp(DurationIn time);
        void setTimestamp(Nanoseconds time);

//         /// Sets a new week number for the timestamp.
//         ///
//         /// The resulting time since epoch is calculated using the old time of week and 
//         /// the new week number.
//         ///
//         /// Example usage:
//         ///     // The timestamp will now be different! It will be the result from setting 
//         ///     // the new week number to 4.
//         ///     Seconds timestamp = TimestampState{}.getTimestamp<Seconds>();
//         ///     setWeek(timestamp, 4);
//         template<typename D>
//         void setWeek(D &timestamp, int week);

        /// Returns raw time since the start of the timestamp's current week.
        template<typename DurationOut>
        DurationOut getTimeOfWeek();
        Nanoseconds getTimeOfWeek();

        /// Sets a new time of week for the timestamp.
        ///
        /// The resulting time since epoch is calculated using the old week number and 
        /// the new time of week.
        template<typename DurationIn> 
        void setTimeOfWeek(DurationIn time);
        void setTimeOfWeek(Nanoseconds time);

//         /// Returns whether two timestamps have diverged from each other.
//         ///
//         /// Timestamps are considered diverged if they differ by one or more units of the 
//         /// comparison duration.
//         /// 
//         /// Example usage:
//         ///     // Timestamps are different by one second.
//         ///     Seconds timestamp1{2};
//         ///     Seconds timestamp2{1};
//         ///     bool changed1 = timeChanged<Seconds>(timestamp1, timestamp2);
//         ///     bool changed2 = timeChanged<Seconds>(timestamp2, timestamp1);
//         ///     // ---> changed1 is true.
//         ///
//         ///     // Timestamps are different by more than one nanosecond.
//         ///     Seconds timestamp1{2}; // 2,000,000,000 nanoseconds
//         ///     Seconds timestamp2{1}; // 1,000,000,000 nanoseconds
//         ///     bool changed = timeChanged<Nanoseconds>(timestamp1, timestamp2); 
//         ///     // ---> changed is true. 
//         ///     // ---> Comparison is done in nanoseconds instead of seconds.
//         ///
//         ///     // Timestamps aren't different by a week or more.
//         ///     Seconds timestamp1{604800}; // 1 week
//         ///     Seconds timestamp2{604799}; // 1 second less than a week
//         ///     bool changed = timeChanged<Weeks>(timestamp1, timestamp2); 
//         ///     // ---> changed is false. 
//         ///     // ---> Comparison is done in weeks.
//         template<typename DCompare, typename D1, typename D2> 
//         bool timeChanged(const D1 &timestamp1, const D2 &timestamp2);

//         /// Casts the timestamp duration to the given arithmetic type.
//         ///
//         /// Optionally performs a duration cast on the timestamp duration before casting 
//         /// to the arithmetic type.
//         ///
//         /// Example usage:
//         ///     // Cast seconds to int.
//         ///     Seconds timestamp{123};
//         ///     int time_casted = castTime<int>(timestamp);    
//         ///     ---> time_casted = 123.
//         ///
//         ///     // Cast nanoseconds to seconds to int.
//         ///     Nanoseconds timestamp{1000000000}; // This equals one second.
//         ///     int time_casted = castTime<int, Seconds>(timestamp);
//         ///     ---> time_casted = 1.
//         template<typename T, typename DCast, typename DIn> 
//         T castTime(const DIn &timestamp);
//         template<typename T, typename D> 
//         T castTime(const D &timestamp);

    private:
        const TimeStandard &m_standard;
        Nanoseconds m_timestamp{0};
        
        // Throws std::invalid_argument if invalid.
        template<typename DurationIn>
        void validateInputTime(const DurationIn &time);
        void validateInputTime(const Nanoseconds &time);
        template<typename DurationIn>
        void validateInputTimeOfWeek(const DurationIn &time);
        void validateInputTimeOfWeek(const Nanoseconds &time);
    };


    /**************************************************************************************/
    /* NOTE: The following are definitions for all template declarations above. There are */
    /*       no new declarations following this statement.                                */
    /**************************************************************************************/

    template<typename DurationIn> 
    inline TimestampExperimental::TimestampExperimental(const TimeStandard &standard, DurationIn time) :
        m_standard(standard)
    {
        validateInputTime(time);
        m_timestamp = time;
    }

    template<typename DurationOut> 
    inline DurationOut TimestampExperimental::getTimestamp()
    {
        return std::chrono::duration_cast<DurationOut>(getTimestamp());
    }

    template<typename DurationIn>
    inline void TimestampExperimental::setTimestamp(DurationIn time)
    {
        setTimestamp(std::chrono::duration_cast<Nanoseconds>(time));
    }
    
    template<typename DurationOut>
    inline DurationOut TimestampExperimental::getTimeOfWeek()
    {
        return std::chrono::duration_cast<DurationOut>(getTimeOfWeek());
    }

    template<typename DurationIn>
    inline void TimestampExperimental::setTimeOfWeek(DurationIn time)
    {
        setTimeOfWeek(std::chrono::duration_cast<Nanoseconds>(time));
    }

//     template<typename DCompare, typename D1, typename D2> 
//     inline bool TimestampExperimental::timeChanged(const D1 &timestamp1, const D2 &timestamp2)
//     {
//         if (timestamp1 >= timestamp2)
//             return (timestamp1 - timestamp2 >= DCompare{1}) ? true : false;

//         return (timestamp2 - timestamp1 >= DCompare{1}) ? true : false;
//     }

//     template<typename T, typename DCast, typename DIn> 
//     inline T TimestampExperimental::castTime(const DIn &timestamp)
//     {
//         return castTime<T>(duration_cast<DCast>(timestamp));
//     }

//     template<typename T, typename D> 
//     inline T TimestampExperimental::castTime(const D &timestamp)
//     {
//         return static_cast<T>(timestamp.count());
//     }
    
//     template<typename D>
//     inline void TimestampExperimental::setWeek(D &timestamp, int week)
//     {
//         assert (week > 0); 
//         if (week <= 0) return;

//         timestamp = duration_cast<D>(Weeks(week) + getTimeOfWeek(timestamp));
//     }

    template<typename DurationIn>
    void TimestampExperimental::validateInputTime(const DurationIn &time)
    {
        validateInputTime(std::chrono::duration_cast<Nanoseconds>(time));
    }

    template<typename DurationIn>
    void TimestampExperimental::validateInputTimeOfWeek(const DurationIn &time)
    {
        validateInputTimeOfWeek(std::chrono::duration_cast<Nanoseconds>(time));
    }
} // namespace mip