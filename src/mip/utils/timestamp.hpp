#pragma once

#include <assert.h>
#include <chrono>
#include <string>

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
    template<typename Standard>
    class TimestampExperimental
    {
    public:
        /// Calls synchronize().
        TimestampExperimental();

        /// Manually sets time since epoch.
        template<typename DurationIn> 
        TimestampExperimental(DurationIn time);

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

    private:
        const Standard m_standard;
        Nanoseconds m_timestamp{0};
    };
} // namespace mip
