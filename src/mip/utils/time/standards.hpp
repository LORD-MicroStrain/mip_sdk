#pragma once

#include "mip/utils/time/durations.hpp"

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
        Nanoseconds now() const override;
        Nanoseconds convertToBase(Nanoseconds time) const override;
        Nanoseconds convertFromBase(Nanoseconds time) const override;
    };
    
    struct GpsTime : TimeStandard 
    {
        Nanoseconds now() const override;
        Nanoseconds convertToBase(Nanoseconds time) const override;
        Nanoseconds convertFromBase(Nanoseconds time) const override;
    };
} // namespace mip