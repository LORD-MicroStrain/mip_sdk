#pragma once

#include "mip/utils/time_durations.hpp"

namespace mip
{
    // TODO: Add more comprehensive documentation.
    /// Extend this to create a new time standard.
    struct TimeStandard
    {
        virtual Nanoseconds now() const = 0;
        virtual Nanoseconds convertToBase(Nanoseconds time) const = 0;
        virtual Nanoseconds convertFromBase(Nanoseconds time) const = 0;
    };
    
    /** Common standards ****************************************************************/

    struct UnixTime : TimeStandard
    {

    };

    struct GpsTime : TimeStandard
    {

    };
} // namespace mip