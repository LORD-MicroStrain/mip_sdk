#pragma once

#include <stdexcept>

#include "mip/utils/time/durations.hpp"
#include "mip/utils/time/standards.hpp"

namespace mip
{
    // TODO: Change name to Timestamp when old one is removed throughout mip sdk.
    /// Manages a timestamp in nanoseconds since epoch.
    ///
    /// Provides various unit of time conversions, timestamp comparisons + tracking, synchronizing 
    /// to a coordinated time standard, etc.
    /// 
    /// Notes:
    /// ----------------------------------------------------------------------------------
    ///     * Currently supports std::chrono:duration for time inputs.
    ///     * Template parameters with 'Duration' in their names represent chrono durations.
    /// ----------------------------------------------------------------------------------
    class TimestampExperimental
    {
    public:
        TimestampExperimental() = delete;

        /// Calls synchronize()
        ///
        /// @param[in] standard Time standard to reference (ex: Unix time). NOTE: This
        ///                     will cause a segmentation fault if it goes out of scope!
        ///
        TimestampExperimental(const mip::TimeStandard &standard);

        /// Manually sets time since epoch.
        ///
        /// @param[in] standard   Time standard to reference (ex: Unix time). NOTE: This
        ///                       will cause a segmentation fault if it goes out of scope!
        /// @param[in] time       Manual starting time since epoch to set.
        ///
        /// @throws std::invalid_argument If time > 0 nanoseconds.
        ///
        template<class DurationIn> 
        TimestampExperimental(const mip::TimeStandard &standard, DurationIn time);

        /// Synchronizes time since epoch with the time standard's time since epoch.
        ///
        /// Continuously call to keep timestamp up-to-date with the time standard.
        ///
        void synchronize();

        /// Increments time since epoch by the time difference between two timestamps.
        ///
        /// Useful for incrementing a manual timestamp with respect to an actual coordinated
        /// time standard, among other things. 
        ///
        /// @param reference_new Timestamp to use as up-to-date reference.
        /// @param reference_old Timestamp to use as old reference to subtract from up-to-date reference.
        ///
        /// @throws std:invalid_argument If reference > old.
        ///
        /// @code
        /// // Assuming:
        /// // * timestamp = 5 nanoseconds since epoch
        /// // * old = 500 nanoseconds since epoch
        /// // * reference = 600 nanoseconds since epoch
        ///
        /// // Outputs 5 nanoseconds
        /// timestamp.getTimestamp();
        ///
        /// // Increment: 5 ns + (600 ns - 500 ns) = 105 nanoseconds
        /// timestamp.increment(reference, old); 
        ///
        /// // Now outputs 105 nanoseconds.
        /// timestamp.getTimestamp();
        /// @endcode
        ///
        void increment(const TimestampExperimental &reference_new, const TimestampExperimental &reference_old);

        /// Returns whether at least one time duration has passed since reference timestamp.
        ///
        /// In simpler terms, this checks whether reference - old >= 1 duration. Useful for
        /// tracking things such as the time since an event started.
        ///
        /// @param[in] reference Old timestamp to compare against (ex: when event was triggerd).
        ///
        /// @throws std::invalid_argument If timestamp < reference.
        ///
        /// @code
        /// // Assuming:
        /// // * timestamp = 1 second + 1 nanosecond since epoch.
        /// // * reference = 1 second - 1 nanosecond since epoch.
        ///
        /// // False since it hasn't been a full second since the reference.
        /// bool elapsed = timestamp.timeElapsed<mip::Seconds>(reference);
        ///
        /// // Assuming:
        /// // * timestamp = 1 second + 1 nanosecond since epoch.
        /// // * reference = 1 nanoseconds since epoch.
        ///
        /// // True since it has been a full second since the reference.
        /// bool elapsed = timestamp.timeElapsed<mip::Seconds>(reference);
        /// @endcode
        ///
        template<typename DurationElapsed = Nanoseconds>
        bool timeElapsed(const TimestampExperimental &reference);

        // TODO: Update documentation.
        /// Returns whether timestamp has entered a new time duration since reference timestamp.
        ///
        ///
        template<typename DurationChanged = Nanoseconds>
        bool timeChanged(const TimestampExperimental &reference);

        /// Sets raw time since epoch for the timestamp.
        ///
        /// @throws std::invalid_argument If time < 0 nanoseconds.
        ///
        template<typename DurationIn>
        void setTimestamp(DurationIn time);
        void setTimestamp(Nanoseconds time);
        void setTimestamp(const TimestampExperimental &from);

        /// Sets a new week number for the timestamp.
        ///
        /// The resulting time since epoch is calculated using the old time of week and 
        /// the new week number.
        ///
        /// @throws std::invalid_argument If week < 0.
        ///
        void setWeek(Weeks week);

        /// Sets a new time of week for the timestamp.
        ///
        /// The resulting time since epoch is calculated using the old week number and 
        /// the new time of week.
        ///
        /// @throws std::invalid_argument If time < 0 nanoseconds, or if time >= 1 week.
        ///
        template<typename DurationIn> 
        void setTimeOfWeek(DurationIn time);
        void setTimeOfWeek(Nanoseconds time);

        /// Returns raw time since epoch.
        template<typename DurationOut> 
        DurationOut getTimestamp() const;
        Nanoseconds getTimestamp() const;
        
        /// Returns raw time since the start of the timestamp's current week.
        template<typename DurationOut>
        DurationOut getTimeOfWeek();
        Nanoseconds getTimeOfWeek();

    private:
        const mip::TimeStandard &m_standard;
        Nanoseconds m_timestamp{0};
    };


    /**************************************************************************************/
    /* NOTE: The following are definitions for all template declarations above. There are */
    /*       no new declarations following this statement.                                */
    /**************************************************************************************/

    template<class DurationIn> 
    inline TimestampExperimental::TimestampExperimental(const mip::TimeStandard &standard, DurationIn time) :
        m_standard(standard)
    {
        if (time < mip::Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        m_timestamp = time;
    }
    
    template<typename DurationElapsed>
    inline bool TimestampExperimental::timeElapsed(const TimestampExperimental &reference)
    {
        const Nanoseconds m_reference = reference.getTimestamp();
        if (m_timestamp < m_reference)
        {
            throw std::invalid_argument("Timestamp < reference timestamp.");
        }
        
        return m_timestamp - m_reference >= DurationElapsed(1);
    }

    template<typename DurationChanged>
    inline bool TimestampExperimental::timeChanged(const TimestampExperimental &reference)
    {
        const Nanoseconds m_reference = reference.getTimestamp();
        if (m_timestamp < m_reference)
        {
            throw std::invalid_argument("Timestamp < reference timestamp.");
        }

        return std::chrono::duration_cast<DurationChanged>(m_timestamp) > std::chrono::duration_cast<DurationChanged>(m_reference);
    }

    template<typename DurationIn>
    inline void TimestampExperimental::setTimestamp(DurationIn time)
    {
        setTimestamp(std::chrono::duration_cast<Nanoseconds>(time));
    }

    template<typename DurationIn>
    inline void TimestampExperimental::setTimeOfWeek(DurationIn time)
    {
        setTimeOfWeek(std::chrono::duration_cast<Nanoseconds>(time));
    }
    
    template<typename DurationOut> 
    inline DurationOut TimestampExperimental::getTimestamp() const
    {
        return std::chrono::duration_cast<DurationOut>(getTimestamp());
    }

    template<typename DurationOut>
    inline DurationOut TimestampExperimental::getTimeOfWeek()
    {
        if (DurationOut(1) >= Weeks(1))
        {
            throw std::invalid_argument("Duration >= one week.");            
        }

        return std::chrono::duration_cast<DurationOut>(getTimeOfWeek());
    }
} // namespace mip
