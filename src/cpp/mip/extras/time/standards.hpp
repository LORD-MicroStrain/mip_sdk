#pragma once

#include "mip/extras/time/durations.hpp"

namespace mip
{
    /// Extend this to create a new time standard.
    struct TimeStandard
    {
        /// Should return the currently most up-to-date time duration in this standard.
        virtual Nanoseconds now() const = 0;

        /// Should convert the given time duration in this standard to the corresponding time in
        /// the base standard.
        virtual Nanoseconds convertToBase(Nanoseconds time) const = 0;

        /// Should convert the given time duration in the base standard to the corresponding
        /// time in this standard.
        virtual Nanoseconds convertFromBase(Nanoseconds time) const = 0;
    };

    /** Common standards ****************************************************************/

    struct UnixTime : TimeStandard
    {
        Nanoseconds now() const override;
        Nanoseconds convertToBase(Nanoseconds time) const override;
        Nanoseconds convertFromBase(Nanoseconds time) const override;
    };

    struct GpsTime : TimeStandard
    {
        Nanoseconds now() const override;
        Nanoseconds convertToBase(Nanoseconds time) const override;
        Nanoseconds convertFromBase(Nanoseconds time) const override;

    private:
        // Accounts for epoch gap and leap seconds.
        const mip::Nanoseconds epoch_difference{Seconds(315964800 - 18)};
    };
} // namespace mip