#pragma once

#include <assert.h>
#include <chrono>

namespace mip
{
    using std::chrono::duration_cast;
    using Nanoseconds = std::chrono::nanoseconds;
    using Microseconds = std::chrono::microseconds;
    using Milliseconds = std::chrono::milliseconds;
    using Seconds = std::chrono::seconds;
    using Minutes = std::chrono::minutes;
    using Hours = std::chrono::hours;

#if _HAS_CXX20
    using Days = std::chrono::days;
    using Weeks = std::chrono::weeks;
    using Years = std::chrono::years;
#else
    #include <ratio>
    using Days = std::chrono::duration<int, std::ratio<86400>>;
    using Weeks = std::chrono::duration<int, std::ratio<604800>>;
    using Years = std::chrono::duration<int, std::ratio<31556952>>;
#endif // _HAS_CXX20

    // TODO: Update documentation.

    /// Manages a timestamp that increments based on an underlying time system.
    ///
    /// Notes:
    /// ---------------------------------------------------------------------------------
    ///     * Currently supports GPS clock (only) for the underlying time system.
    ///     * Might add support for different time systems in the future.
    ///     * Currently supports std::chrono:duration (only) for timestamp inputs.
    /// ---------------------------------------------------------------------------------
    class TimestampManager
    {
    public:
        TimestampManager() {}
        TimestampManager(long long start_time) : 
            m_timestamp(Nanoseconds(start_time)) {}

        /// Synchronizes timestamp to the current time in the underlying time system.
        void synchronize();

        /// Returns time since epoch.
        template<typename Duration> Duration getTimestamp();
        Nanoseconds getTimestamp();

        /// Returns time since the start of the current week (of the timestamp).
        template<typename Duration> Duration getTimeOfWeek();
        Nanoseconds getTimeOfWeek();

        /// Returns whether two timestamps have diverged from each other.
        ///
        /// Timestamps are considered diverged if they differ by one or more units of the 
        /// comparison duration.
        /// 
        /// Example usage:
        ///     // Timestamps are different by one second.
        ///     Seconds timestamp1{2};
        ///     Seconds timestamp2{1};
        ///     bool changed1 = timeChanged<Seconds>(timestamp1, timestamp2);
        ///     bool changed2 = timeChanged<Seconds>(timestamp2, timestamp1);
        ///     // ---> changed1 is true.
        ///
        ///     // Timestamps are different by more than one nanosecond.
        ///     Seconds timestamp1{2}; // 2,000,000,000 nanoseconds
        ///     Seconds timestamp2{1}; // 1,000,000,000 nanoseconds
        ///     bool changed = timeChanged<Nanoseconds>(timestamp1, timestamp2); 
        ///     // ---> changed is true. 
        ///     // ---> Comparison is done in nanoseconds instead of seconds.
        ///
        ///     // Timestamps aren't different by a week or more.
        ///     Seconds timestamp1{604800}; // 1 week
        ///     Seconds timestamp2{604799}; // 1 second less than a week
        ///     bool changed = timeChanged<Weeks>(timestamp1, timestamp2); 
        ///     // ---> changed is false. 
        ///     // ---> Comparison is done in weeks.
        template<typename DCompare, typename D1, typename D2> 
        bool timeChanged(const D1 &timestamp1, const D2 &timestamp2);

        /// Sets a new week number for the timestamp.
        ///
        /// The resulting time since epoch is calculated using the old time of week and 
        /// the new week number.
        ///
        /// Example usage:
        ///     // The timestamp will now be different! It will be the result from setting 
        ///     // the new week number to 4.
        ///     Seconds timestamp = TimestampState{}.getTimestamp<Seconds>();
        ///     setWeek(timestamp, 4);
        template<typename D>
        void setWeek(D &timestamp, int week);

        /// Sets a new time of week for the timestamp.
        ///
        /// The resulting time since epoch is calculated using the old week number and 
        /// the new time of week.
        ///
        /// Example usage:
        ///     // Setting time of week with the same unit duration as the timestamp.
        ///     Seconds timestamp = TimestampState{}.getTimestamp<Seconds>();
        ///     setTimeOfWeek<Seconds>(timestamp, 300);
        ///     // ---> The timestamp will now be different! It will be the result from 
        ///     //      setting the new time of week to 300 seconds. 
        ///
        ///     // Setting time of week with a different unit duration from the timestamp.
        ///     Nanoseconds timestamp = TimestampState{}.getTimestamp<Nanoseconds>();
        ///     setTimeOfWeek<Seconds>(timestamp, 6);
        ///     // ---> The timestamp will now be the result from setting the new time of 
        ///     //      week to 6 seconds (6,000,000,000 nanoseconds).
        template<typename DTimeSet, typename DIn> 
        void setTimeOfWeek(DIn &timestamp, int time);

        /// Casts the timestamp duration to the given arithmetic type.
        ///
        /// Optionally performs a duration cast on the timestamp duration before casting 
        /// to the arithmetic type.
        ///
        /// Example usage:
        ///     // Cast seconds to int.
        ///     Seconds timestamp{123};
        ///     int time_casted = castTime<int>(timestamp);    
        ///     ---> time_casted = 123.
        ///
        ///     // Cast nanoseconds to seconds to int.
        ///     Nanoseconds timestamp{1000000000}; // This equals one second.
        ///     int time_casted = castTime<int, Seconds>(timestamp);
        ///     ---> time_casted = 1.
        template<typename T, typename DCast, typename DIn> 
        T castTime(const DIn &timestamp);
        template<typename T, typename D> 
        T castTime(const D &timestamp);

    private:
        Nanoseconds m_timestamp{0};
    };
    
    // TODO: Add static synchronizer function.


    /**************************************************************************************/
    /* NOTE: The following are definitions for all template declarations above. There are */
    /*       no new declarations following this statement.                                */
    /**************************************************************************************/

    template<typename Duration> inline Duration TimestampManager::getTimestamp()
    {
        return duration_cast<Duration>(m_timestamp);
    }

    inline Nanoseconds TimestampManager::getTimestamp()
    {
        return m_timestamp;
    }

    template<typename Duration> inline Duration TimestampManager::getTimeOfWeek()
    {
        if (m_timestamp <= Weeks(1)) 
        {
            return getTimestamp<Duration>();
        }

        return duration_cast<Duration>(m_timestamp - duration_cast<Weeks>(m_timestamp));
    }

    template<typename DCompare, typename D1, typename D2> 
    inline bool TimestampManager::timeChanged(const D1 &timestamp1, const D2 &timestamp2)
    {
        if (timestamp1 >= timestamp2)
            return (timestamp1 - timestamp2 >= DCompare{1}) ? true : false;

        return (timestamp2 - timestamp1 >= DCompare{1}) ? true : false;
    }

    template<typename T, typename DCast, typename DIn> 
    inline T TimestampManager::castTime(const DIn &timestamp)
    {
        return castTime<T>(duration_cast<DCast>(timestamp));
    }

    template<typename T, typename D> 
    inline T TimestampManager::castTime(const D &timestamp)
    {
        return static_cast<T>(timestamp.count());
    }
    
    template<typename D>
    inline void TimestampManager::setWeek(D &timestamp, int week)
    {
        assert (week > 0); 
        if (week <= 0) return;

        timestamp = duration_cast<D>(Weeks(week) + getTimeOfWeek(timestamp));
    }

    template<typename DTimeSet, typename DIn> 
    inline void TimestampManager::setTimeOfWeek(DIn &timestamp, int time)
    {
        assert (time > 0);
        if (time <= 0) return;

        timestamp = duration_cast<DIn>(duration_cast<Weeks>(timestamp) + DTimeSet(time));
    }
}// namespace mip