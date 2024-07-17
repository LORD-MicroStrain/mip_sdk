#pragma once

#include <assert.h>
#include <chrono>
#include <string>
#include <stdexcept>

#include "mip/utils/time_durations.hpp"
#include "mip/utils/time_standard.hpp"

namespace mip
{
    // TODO: Update documentation.
    // TODO: Change name to Timestamp when old one is removed throughout mip sdk.
    /// Manages a timestamp in nanoseconds since epoch.
    ///
    /// Notes:
    /// ----------------------------------------------------------------------------------
    ///     * Currently supports GPS clock (only) for the underlying time system.
    ///     * Might add support for different time systems in the future.
    ///     * Currently supports std::chrono:duration (only) for timestamp inputs.
    ///     * TimeStandard reference should not go out of scope. It will cause a segmentation fault if it does!
    /// ----------------------------------------------------------------------------------
    class TimestampExperimental
    {
    public:
        TimestampExperimental() = delete;

        /// Calls synchronize().
        TimestampExperimental(const mip::TimeStandard &standard);

        /// Manually sets time since epoch.
        template<class DurationIn> 
        TimestampExperimental(const mip::TimeStandard &standard, DurationIn time);

        /// Synchronizes timestamp to a coordinated time standard. Continuously call this 
        /// method to keep the timestamp up to date with the time standard.
        void synchronize();
        
        // TODO: Add documentation.
        void increment(const TimestampExperimental &reference_synced, const TimestampExperimental &reference_old);

        /// Returns raw time since epoch.
        template<typename DurationOut> 
        DurationOut getTimestamp() const;
        Nanoseconds getTimestamp() const;
        
        // Sets raw time since epoch.
        template<typename DurationIn>
        void setTimestamp(DurationIn time);
        void setTimestamp(Nanoseconds time);

        /// Sets a new week number for the timestamp.
        ///
        /// The resulting time since epoch is calculated using the old time of week and 
        /// the new week number.
        void setWeek(Weeks week);

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
        
        // TODO: Update documentation.
        /// Returns whether the timestamp has diverged since a reference timestamp.
        ///
        /// Timestamps are considered diverged if they differ by one or more units of the 
        /// comparison duration.
        template<typename DurationElapsed = Nanoseconds>
        bool timeElapsed(const TimestampExperimental &reference);

        // TODO: Update documentation.
        /// Returns whether the timestamp has entered a new duration interval since a 
        /// reference timestamp.
        template<typename DurationChanged = Nanoseconds>
        bool timeChanged(const TimestampExperimental &reference);

        // TODO: Update documentation.
        /// Casts a timestamp duration to the given arithmetic type.
        template<typename T, typename DurationIn>
        T castTime(const DurationIn &time);

        // TODO: Move to Timestamp.
        // TODO: Add duration changing.
        // template<typename DurationIn, typename DurationOut> 
        //     DurationOut convert(std::uint64_t time, TimeStandard to, 
        //     TimeStandard from); 
        // Nanoseconds setTimestamp(std::uint64_t time, TimeStandard to, 
        //     TimeStandard from)
        // Nanoseconds setTimestamp(std::uint64_t time, TimeStandard from)
        template<class DurationOut = Nanoseconds>
        DurationOut convert(const TimestampExperimental &reference)
        {
            Nanoseconds converted = m_standard.convertToBase(reference.getTimestamp());
            if (converted < Nanoseconds(0))
            {
                return DurationOut(0);
            }
            
            return std::chrono::duration_cast<DurationOut>(converted);
        }

    private:
        const mip::TimeStandard &m_standard;
        Nanoseconds m_timestamp{0};
    };


    /**************************************************************************************/
    /* NOTE: The following are definitions for all template declarations above. There are */
    /*       no new declarations following this statement.                                */
    /**************************************************************************************/

    inline TimestampExperimental::TimestampExperimental(const mip::TimeStandard &standard) :
        m_standard(standard)
    {
        synchronize();
    }

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

    inline void TimestampExperimental::synchronize()
    {
        m_timestamp = m_standard.now();
    }

    inline void TimestampExperimental::increment(const TimestampExperimental &reference_synced, const TimestampExperimental &reference_old)
    {
        mip::Nanoseconds m_synced = reference_synced.getTimestamp();
        mip::Nanoseconds m_old = reference_old.getTimestamp();
        if (m_synced < m_old)
        {
            throw std::invalid_argument("Reference timestamp < old timestamp.");
        }
        
        m_timestamp += (m_synced - m_old);
    }

    template<typename DurationOut> 
    inline DurationOut TimestampExperimental::getTimestamp() const
    {
        return std::chrono::duration_cast<DurationOut>(getTimestamp());
    }

    inline Nanoseconds TimestampExperimental::getTimestamp() const
    {
        return m_timestamp;
    }

    template<typename DurationIn>
    inline void TimestampExperimental::setTimestamp(DurationIn time)
    {
        setTimestamp(std::chrono::duration_cast<Nanoseconds>(time));
    }
    
    inline void TimestampExperimental::setTimestamp(Nanoseconds time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        m_timestamp = time;
    }

    inline void TimestampExperimental::setWeek(Weeks week)
    {
        if (week < Weeks(0))         
        {
            throw std::invalid_argument("Week < 0.");
        }

        m_timestamp = week + getTimeOfWeek();
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

    inline Nanoseconds TimestampExperimental::getTimeOfWeek()
    {
        return m_timestamp % Weeks(1);
    }

    template<typename DurationIn>
    inline void TimestampExperimental::setTimeOfWeek(DurationIn time)
    {
        setTimeOfWeek(std::chrono::duration_cast<Nanoseconds>(time));
    }
    
    inline void TimestampExperimental::setTimeOfWeek(Nanoseconds time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("Time of week < 0.");
        }
        if (time >= Weeks(1))
        {
            throw std::invalid_argument("Time of week >= one week.");
        }

        m_timestamp = std::chrono::duration_cast<Weeks>(m_timestamp) + time;
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

    template<typename T, typename DurationIn>
    T TimestampExperimental::castTime(const DurationIn &time)
    {
        if (time < mip::Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        return static_cast<T>(time.count());
    }
} // namespace mip
